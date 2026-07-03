#!/usr/bin/env python3
"""
grid_projector_node.py  —  Stage 4 deployment, Step 1
========================================================
Purpose
-------
Bridges Stage 3's fusion output (/global_map, a dynamic-canvas
nav_msgs/OccupancyGrid with ROS-standard 0/100/-1 encoding) to the
FIXED canvas convention that multi_robot_env.py and the trained policy
expect:

    canvas:    400 x 400 cells
    world:     x,y in [-10, 10]   (origin fixed at (-10,-10))
    res:       0.05 m/cell
    encoding:  FREE=0  OCCUPIED=1  UNKNOWN=-1   (NOT ROS's 0/100/-1)

This node does NOT modify central_server_unified.py or its merge
methods, and does NOT change /global_map's semantics. It only adds a
read-only consumer + republisher. Stage 3 fusion-method comparison
results (which depend on the dynamic canvas's unknown_pct/occupied_pct/
wall_sharpness metrics) are completely unaffected.

Outputs
-------
  /training_grid          std_msgs/Int8MultiArray   (400,400) flattened,
                           row-major, values in {-1,0,1}. This is the
                           exact array multi_robot_env.w2g()-style
                           cropping expects. Consumed later by the
                           coordination node (Step 3).

  /training_grid_rviz     nav_msgs/OccupancyGrid    same data, re-encoded
                           back to ROS convention (0/100/-1) purely so it
                           can be viewed in RViz next to /global_map for
                           the visual sanity check described in the plan.
                           NOT consumed by the policy — debug only.

Sanity check workflow (do this before writing anything else)
--------------------------------------------------------------
  1. Run this node alongside the existing Stage 3 launch.
  2. Add both /global_map and /training_grid_rviz as Map displays in
     RViz (set Fixed Frame to "world").
  3. They should look like the same building outline. /training_grid_rviz
     will appear anchored to the full 20x20 arena and mostly "unknown"
     (grey) outside whatever /global_map's current dynamic bounding box
     covers — that is expected and matches what the training env's
     shared_map looks like at episode start.
  4. If the wall shapes are rotated, mirrored, or offset relative to
     /global_map, the regrid math below has a bug — stop and debug
     before going anywhere near Step 2 (Nav2 smoke test).

Run
---
  python3 grid_projector_node.py
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int8MultiArray, MultiArrayDimension

# ── Must mirror multi_robot_env.py EXACTLY ──────────────────────────────────
WORLD_MIN  = -10.0
WORLD_MAX  =  10.0
RESOLUTION =  0.05
GRID_SIZE  = 400

FREE = 0
OCCUPIED = 1
UNKNOWN = -1

# Pre-compute the world (x,y) coordinate of every destination cell centre,
# once, at import time. This is what makes the regrid vectorised instead of
# a 400x400 python double-loop on every callback.
_cols = np.arange(GRID_SIZE)
_rows = np.arange(GRID_SIZE)
_DST_WX = WORLD_MIN + _cols * RESOLUTION + RESOLUTION / 2.0      # (400,)
_DST_WY = WORLD_MIN + _rows * RESOLUTION + RESOLUTION / 2.0      # (400,)


def regrid_to_training_canvas(occ_msg: OccupancyGrid) -> np.ndarray:
    """
    Re-grid a dynamic-canvas ROS OccupancyGrid onto the fixed training
    canvas, and remap encoding from ROS convention (0=free,100=occ,-1=unk)
    to training convention (0=free,1=occ,-1=unk).

    Returns: np.int8 array, shape (400, 400), row-major, where
             row 0 = WORLD_MIN end of y-axis (matches w2g()/g2w() in
             multi_robot_env.py).
    """
    src = np.array(occ_msg.data, dtype=np.int16).reshape(
        occ_msg.info.height, occ_msg.info.width)

    ox = occ_msg.info.origin.position.x
    oy = occ_msg.info.origin.position.y
    res = occ_msg.info.resolution

    # For every destination cell, find which source cell it falls in.
    src_col = np.floor((_DST_WX - ox) / res).astype(np.int32)   # (400,)
    src_row = np.floor((_DST_WY - oy) / res).astype(np.int32)   # (400,)

    valid_col = (src_col >= 0) & (src_col < occ_msg.info.width)
    valid_row = (src_row >= 0) & (src_row < occ_msg.info.height)

    # Build full (400,400) index grids, clip out-of-range to 0 (will be
    # masked out right after — clipping only avoids an out-of-bounds
    # index, the actual value at clipped positions is discarded).
    col_grid, row_grid = np.meshgrid(src_col, src_row)          # (400,400) each
    valid = np.meshgrid(valid_col, valid_row)
    valid_mask = valid[0] & valid[1]

    col_clip = np.clip(col_grid, 0, occ_msg.info.width - 1)
    row_clip = np.clip(row_grid, 0, occ_msg.info.height - 1)

    sampled = src[row_clip, col_clip]                            # (400,400)

    dst = np.full((GRID_SIZE, GRID_SIZE), UNKNOWN, dtype=np.int8)
    occ_mask  = valid_mask & (sampled == 100)
    free_mask = valid_mask & (sampled == 0)
    dst[occ_mask]  = OCCUPIED
    dst[free_mask] = FREE
    # everything else (unmapped cells, or sampled == -1) stays UNKNOWN

    return dst


def training_grid_to_rviz_msg(grid: np.ndarray, stamp) -> OccupancyGrid:
    """Re-encode the fixed training-convention grid back to ROS convention
    purely for RViz display. Debug-only, never consumed by the policy."""
    ros_data = np.full(grid.shape, -1, dtype=np.int8)
    ros_data[grid == OCCUPIED] = 100
    ros_data[grid == FREE] = 0

    msg = OccupancyGrid()
    msg.header.frame_id = "world"
    msg.header.stamp = stamp
    msg.info.resolution = RESOLUTION
    msg.info.width = GRID_SIZE
    msg.info.height = GRID_SIZE
    msg.info.origin.position.x = WORLD_MIN
    msg.info.origin.position.y = WORLD_MIN
    msg.info.origin.orientation.w = 1.0
    msg.data = ros_data.flatten().tolist()
    return msg


def training_grid_to_multiarray(grid: np.ndarray) -> Int8MultiArray:
    msg = Int8MultiArray()
    dim0 = MultiArrayDimension(label="row", size=GRID_SIZE, stride=GRID_SIZE * GRID_SIZE)
    dim1 = MultiArrayDimension(label="col", size=GRID_SIZE, stride=GRID_SIZE)
    msg.layout.dim = [dim0, dim1]
    msg.layout.data_offset = 0
    msg.data = grid.flatten().tolist()
    return msg


class GridProjectorNode(Node):
    def __init__(self):
        super().__init__("grid_projector_node")

        self.declare_parameter("input_topic", "/global_map")
        self.declare_parameter("publish_rate_hz", 2.0)

        input_topic = self.get_parameter("input_topic").value
        rate = self.get_parameter("publish_rate_hz").value

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE

        self._latest_msg = None

        self.sub = self.create_subscription(
            OccupancyGrid, input_topic, self._on_global_map, qos)

        self.pub_array = self.create_publisher(
            Int8MultiArray, "/training_grid", 1)
        self.pub_rviz = self.create_publisher(
            OccupancyGrid, "/training_grid_rviz", qos)

        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f"grid_projector_node up | subscribing {input_topic} | "
            f"fixed canvas {GRID_SIZE}x{GRID_SIZE} @ {RESOLUTION} m/cell, "
            f"world [{WORLD_MIN},{WORLD_MAX}]")

    def _on_global_map(self, msg: OccupancyGrid):
        self._latest_msg = msg

    def _tick(self):
        if self._latest_msg is None:
            return
        msg = self._latest_msg

        if msg.info.width == 0 or msg.info.height == 0:
            self.get_logger().warn("Received empty /global_map, skipping.",
                                    throttle_duration_sec=5.0)
            return

        grid = regrid_to_training_canvas(msg)

        stamp = self.get_clock().now().to_msg()
        self.pub_array.publish(training_grid_to_multiarray(grid))
        self.pub_rviz.publish(training_grid_to_rviz_msg(grid, stamp))

        # Lightweight sanity log so you can eyeball coverage % without RViz
        total = grid.size
        unk_pct = 100.0 * np.sum(grid == UNKNOWN) / total
        occ_pct = 100.0 * np.sum(grid == OCCUPIED) / total
        free_pct = 100.0 * np.sum(grid == FREE) / total
        self.get_logger().info(
            f"[training_grid] unk={unk_pct:.1f}% occ={occ_pct:.1f}% "
            f"free={free_pct:.1f}%",
            throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = GridProjectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()