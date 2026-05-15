#!/usr/bin/env python3
"""
Central Server — Stage 3a Week 3: Probabilistic (Log-Odds Bayesian) Map Fusion

Compared to Week 2 (ICP):
  - No small_gicp / no correction transforms / no alignment timer
  - merge_and_publish replaced with log_odds_merge() — Bayesian cell fusion
  - All metrics infrastructure is identical to Week 2 for fair Stage 3b comparison

Log-odds model
--------------
  L(cell) = log( P(occupied) / P(free) )

  Per-observation updates (per robot, per merge cycle):
    occupied  (100) → += L_OCC  = +0.85   (P ≈ 0.70)
    free      (  0) → += L_FREE = -0.40   (P ≈ 0.40)
    unknown   ( -1) → no update

  Clamped to [-L_MAX, +L_MAX] = [-3.5, +3.5]

  Published occupancy:
    P > 0.65  → 100  (occupied)
    P < 0.35  →   0  (free)
    otherwise →  -1  (unknown / conflicted)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from tf2_ros import (Buffer, TransformListener,
                     LookupException, ConnectivityException, ExtrapolationException)
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import numpy as np
import struct
import threading
import json
import os
import time
from datetime import datetime

# ─────────────────────────────────────────────
#  Log-odds sensor model constants
# ─────────────────────────────────────────────
L_OCC  =  0.85   # log-odds update for one occupied observation
L_FREE = -0.40   # log-odds update for one free observation
L_MAX  =  3.50   # clamp: prevents absolute certainty accumulating

P_OCC_THRESH  = 0.65   # sigmoid(L) above this  → publish 100
P_FREE_THRESH = 0.35   # sigmoid(L) below this  → publish 0
# in between → publish -1 (unknown / conflicted)


# ─────────────────────────────────────────────
#  Shared helper functions
#  (identical across baseline, ICP, probabilistic
#   — do not modify, keeps Stage 3b comparison clean)
# ─────────────────────────────────────────────

def pointcloud2_to_xyz(msg):
    offsets = {}
    for field in msg.fields:
        if field.name in ("x", "y", "z"):
            offsets[field.name] = field.offset
    if not all(k in offsets for k in ("x", "y", "z")):
        return np.array([])
    n    = msg.width * msg.height
    step = msg.point_step
    data = msg.data
    xyz  = np.zeros((n, 3), dtype=np.float32)
    for i in range(n):
        base = i * step
        xyz[i, 0] = struct.unpack_from("f", data, base + offsets["x"])[0]
        xyz[i, 1] = struct.unpack_from("f", data, base + offsets["y"])[0]
        xyz[i, 2] = struct.unpack_from("f", data, base + offsets["z"])[0]
    valid = np.isfinite(xyz).all(axis=1)
    return xyz[valid]


def accumulated_cloud_to_occupancy_grid(xyz, resolution=0.05,
                                        z_min=0.1, z_max=2.5,
                                        frame_id="world"):
    if len(xyz) == 0:
        return None
    mask  = (xyz[:, 2] >= z_min) & (xyz[:, 2] <= z_max)
    pts2d = xyz[mask, :2]
    if len(pts2d) == 0:
        return None
    floor_mask = (xyz[:, 2] >= -0.5) & (xyz[:, 2] < z_min)
    floor_pts  = xyz[floor_mask, :2]
    all_pts = np.vstack([pts2d, floor_pts]) if len(floor_pts) > 0 else pts2d
    min_x = all_pts[:, 0].min() - 1.0
    min_y = all_pts[:, 1].min() - 1.0
    max_x = all_pts[:, 0].max() + 1.0
    max_y = all_pts[:, 1].max() + 1.0
    width  = int(np.ceil((max_x - min_x) / resolution))
    height = int(np.ceil((max_y - min_y) / resolution))
    grid = np.full((height, width), -1, dtype=np.int8)
    if len(floor_pts) > 0:
        gx = ((floor_pts[:, 0] - min_x) / resolution).astype(int)
        gy = ((floor_pts[:, 1] - min_y) / resolution).astype(int)
        v  = (gx >= 0) & (gx < width) & (gy >= 0) & (gy < height)
        grid[gy[v], gx[v]] = 0
    gx = ((pts2d[:, 0] - min_x) / resolution).astype(int)
    gy = ((pts2d[:, 1] - min_y) / resolution).astype(int)
    v  = (gx >= 0) & (gx < width) & (gy >= 0) & (gy < height)
    grid[gy[v], gx[v]] = 100
    msg = OccupancyGrid()
    msg.info.resolution = resolution
    msg.info.width      = width
    msg.info.height     = height
    msg.info.origin.position.x    = float(min_x)
    msg.info.origin.position.y    = float(min_y)
    msg.info.origin.orientation.w = 1.0
    msg.header.frame_id = frame_id
    msg.data = grid.flatten().tolist()
    return msg


def _voxel_downsample(xyz, voxel_size=0.05):
    voxels = np.floor(xyz / voxel_size).astype(int)
    _, unique_idx = np.unique(voxels, axis=0, return_index=True)
    return xyz[unique_idx]


def _wall_sharpness(grid_np):
    """
    Mean run-length of occupied pixels in rows and columns.
    Shorter runs = sharper, thinner walls = better map quality.
    Identical to baseline and ICP versions.
    """
    occupied = (grid_np == 100)
    run_lengths = []
    for row in occupied:
        in_run, length = False, 0
        for val in row:
            if val:
                in_run = True
                length += 1
            elif in_run:
                run_lengths.append(length)
                in_run, length = False, 0
        if in_run:
            run_lengths.append(length)
    for col in occupied.T:
        in_run, length = False, 0
        for val in col:
            if val:
                in_run = True
                length += 1
            elif in_run:
                run_lengths.append(length)
                in_run, length = False, 0
        if in_run:
            run_lengths.append(length)
    return float(np.mean(run_lengths)) if run_lengths else 0.0


# ─────────────────────────────────────────────
#  Probabilistic merge  (NEW — replaces the
#  deterministic occupied-wins loop)
# ─────────────────────────────────────────────

def log_odds_merge(maps: dict, resolution: float):
    """
    Merge {robot_name: OccupancyGrid} using Bayesian log-odds fusion.

    Steps
    -----
    1. Compute bounding box covering all robot maps.
    2. Allocate float32 log-odds grid, initialised to 0 (prior P = 0.5).
    3. For each robot map, vectorised numpy scatter:
         occupied cells  → += L_OCC
         free cells      → += L_FREE
         unknown cells   → no update (prior preserved)
    4. Clamp to [-L_MAX, +L_MAX].
    5. sigmoid(log_odds) → probability → threshold → 0 / 100 / -1.

    Why the merge is better than occupied-wins
    ------------------------------------------
    Wall seen by 3 robots:  log-odds ≈ 3×0.85 = +2.55  →  P ≈ 0.93  →  100
    Wall seen by 1 robot:   log-odds ≈ +0.85           →  P ≈ 0.70  →  100
    Free(r1) + Occ(r2):     log-odds ≈ 0.85−0.40 = +0.45  →  P ≈ 0.61
                            → falls in [0.35, 0.65]  →  -1  (conflicted)
    Baseline would publish 100 for that last case (occupied wins blindly).
    """
    if not maps:
        return None

    # 1. Global bounding box
    min_x = min_y =  float("inf")
    max_x = max_y = float("-inf")
    for m in maps.values():
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y
        min_x = min(min_x, ox)
        min_y = min(min_y, oy)
        max_x = max(max_x, ox + m.info.width  * resolution)
        max_y = max(max_y, oy + m.info.height * resolution)

    width  = int(np.ceil((max_x - min_x) / resolution))
    height = int(np.ceil((max_y - min_y) / resolution))

    # 2. Log-odds accumulator
    log_odds = np.zeros((height, width), dtype=np.float32)

    # 3. Per-robot vectorised updates
    for m in maps.values():
        ox    = m.info.origin.position.x
        oy    = m.info.origin.position.y
        mw    = m.info.width
        mh    = m.info.height
        x_off = int(round((ox - min_x) / resolution))
        y_off = int(round((oy - min_y) / resolution))

        data = np.array(m.data, dtype=np.int8).reshape(mh, mw)

        gy, gx = np.meshgrid(np.arange(mh) + y_off,
                              np.arange(mw) + x_off,
                              indexing="ij")               # (mh, mw)

        valid = (gx >= 0) & (gx < width) & (gy >= 0) & (gy < height)

        occ_mask  = valid & (data == 100)
        free_mask = valid & (data == 0)

        log_odds[gy[occ_mask],  gx[occ_mask]]  += L_OCC
        log_odds[gy[free_mask], gx[free_mask]] += L_FREE

    # 4. Clamp
    np.clip(log_odds, -L_MAX, L_MAX, out=log_odds)

    # 5. Sigmoid → probability → occupancy
    prob   = 1.0 / (1.0 + np.exp(-log_odds))
    merged = np.full((height, width), -1, dtype=np.int8)
    merged[prob >  P_OCC_THRESH]  = 100
    merged[prob <  P_FREE_THRESH] = 0

    # 6. Pack into OccupancyGrid
    gmap = OccupancyGrid()
    gmap.header.frame_id              = "world"
    gmap.info.resolution              = resolution
    gmap.info.width                   = width
    gmap.info.height                  = height
    gmap.info.origin.position.x      = min_x
    gmap.info.origin.position.y      = min_y
    gmap.info.origin.orientation.w   = 1.0
    gmap.data = merged.flatten().tolist()
    return gmap


# ─────────────────────────────────────────────
#  Central Server node
# ─────────────────────────────────────────────

class CentralServer(Node):
    def __init__(self):
        super().__init__("central_server")

        # ── ROS parameters ───────────────────────────────────────────────────
        self.declare_parameter("robot_namespaces",     ["robot1", "robot2", "robot3"])
        self.declare_parameter("map_merge_frequency",  1.0)
        self.declare_parameter("resolution",           0.05)
        self.declare_parameter("z_min",                0.1)
        self.declare_parameter("z_max",                2.5)
        self.declare_parameter("max_points_per_robot", 500000)
        self.declare_parameter("metrics_interval_sec", 30.0)
        self.declare_parameter("method_label",         "probabilistic")

        self.robot_namespaces = self.get_parameter("robot_namespaces").value
        self.merge_frequency  = self.get_parameter("map_merge_frequency").value
        self.resolution       = self.get_parameter("resolution").value
        self.z_min            = self.get_parameter("z_min").value
        self.z_max            = self.get_parameter("z_max").value
        self.max_points       = self.get_parameter("max_points_per_robot").value
        self.metrics_interval = self.get_parameter("metrics_interval_sec").value
        self.method_label     = self.get_parameter("method_label").value

        # ── Metrics directory (same structure as baseline and ICP) ────────────
        run_ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.metrics_dir  = f"/tmp/fusion_metrics/{self.method_label}_{run_ts}"
        os.makedirs(self.metrics_dir, exist_ok=True)
        self.metrics_file = os.path.join(self.metrics_dir, "metrics.jsonl")
        self.start_time   = time.time()

        with open(os.path.join(self.metrics_dir, "run_info.json"), "w") as f:
            json.dump({
                "method":          self.method_label,
                "started":         run_ts,
                "resolution":      self.resolution,
                "z_min":           self.z_min,
                "z_max":           self.z_max,
                "robots":          list(self.robot_namespaces),
                "L_OCC":           L_OCC,
                "L_FREE":          L_FREE,
                "L_MAX":           L_MAX,
                "P_OCC_THRESH":    P_OCC_THRESH,
                "P_FREE_THRESH":   P_FREE_THRESH,
            }, f, indent=2)

        self.get_logger().info(
            f"[Metrics] Logging to {self.metrics_dir}  "
            f"L_OCC={L_OCC} L_FREE={L_FREE} L_MAX={L_MAX}  "
            f"P_thresh=({P_FREE_THRESH}, {P_OCC_THRESH})")

        # ── TF ───────────────────────────────────────────────────────────────
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── State ────────────────────────────────────────────────────────────
        self.robot_clouds = {name: np.zeros((0, 3), dtype=np.float32)
                             for name in self.robot_namespaces}
        self.robot_maps        = {}
        self.global_map_latest = None      # for metrics — same as ICP
        self.lock = threading.Lock()

        # ── Publishers ───────────────────────────────────────────────────────
        qos = QoSProfile(depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.global_map_pub = self.create_publisher(OccupancyGrid, "/global_map", qos)
        self.robot_map_pubs = {
            name: self.create_publisher(OccupancyGrid, f"/{name}/map", qos)
            for name in self.robot_namespaces
        }

        # ── Subscribers ──────────────────────────────────────────────────────
        for name in self.robot_namespaces:
            self.create_subscription(
                PointCloud2, f"/{name}/glim/aligned_points",
                lambda msg, n=name: self.pointcloud_callback(msg, n), 10)
            self.get_logger().info(f"Subscribed to /{name}/glim/aligned_points")

        # ── Timers ───────────────────────────────────────────────────────────
        self.merge_timer   = self.create_timer(1.0 / self.merge_frequency,
                                               self.merge_and_publish)
        self.metrics_timer = self.create_timer(self.metrics_interval,
                                               self.log_metrics)

        self.get_logger().info(
            f"Central Server (Probabilistic log-odds fusion) started  "
            f"method={self.method_label}")

    # ── upstream pipeline (identical to baseline and ICP) ────────────────────

    def pointcloud_callback(self, msg, robot_ns):
        try:
            transform = self.tf_buffer.lookup_transform(
                "world", msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))
            msg_world = do_transform_cloud(msg, transform)
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f"TF lookup failed for {robot_ns}: {e}",
                throttle_duration_sec=5.0)
            msg_world = msg

        xyz = pointcloud2_to_xyz(msg_world)
        if len(xyz) == 0:
            return

        with self.lock:
            current  = self.robot_clouds[robot_ns]
            combined = np.vstack([current, xyz]) if len(current) > 0 else xyz
            if len(combined) > self.max_points:
                combined = _voxel_downsample(combined, voxel_size=0.05)
            self.robot_clouds[robot_ns] = combined

        occ = accumulated_cloud_to_occupancy_grid(
            combined, resolution=self.resolution,
            z_min=self.z_min, z_max=self.z_max, frame_id="world")
        if occ is None:
            return
        occ.header.stamp = msg.header.stamp

        with self.lock:
            self.robot_maps[robot_ns] = occ
        self.robot_map_pubs[robot_ns].publish(occ)

    # ── probabilistic merge ───────────────────────────────────────────────────

    def merge_and_publish(self):
        with self.lock:
            if not self.robot_maps:
                return
            maps = dict(self.robot_maps)

        global_map = log_odds_merge(maps, self.resolution)
        if global_map is None:
            return

        global_map.header.stamp = self.get_clock().now().to_msg()
        self.global_map_pub.publish(global_map)

        with self.lock:
            self.global_map_latest = global_map   # snapshot for metrics

    # ── metrics logging (identical structure to baseline and ICP) ─────────────

    def log_metrics(self):
        with self.lock:
            gmap   = self.global_map_latest
            clouds = {n: len(c) for n, c in self.robot_clouds.items()}

        if gmap is None:
            self.get_logger().info("[Metrics] No global map yet, skipping.")
            return

        data         = np.array(gmap.data, dtype=np.int8)
        total        = len(data)
        unknown_pct  = float(np.sum(data == -1)  / total * 100)
        occupied_pct = float(np.sum(data == 100) / total * 100)
        free_pct     = float(np.sum(data == 0)   / total * 100)
        sharpness    = _wall_sharpness(data.reshape(gmap.info.height,
                                                    gmap.info.width))
        elapsed      = time.time() - self.start_time

        record = {
            "timestamp":               datetime.now().isoformat(),
            "elapsed_sec":             round(elapsed, 1),
            "method":                  self.method_label,
            "map_width":               gmap.info.width,
            "map_height":              gmap.info.height,
            "total_cells":             total,
            "unknown_pct":             round(unknown_pct, 2),
            "occupied_pct":            round(occupied_pct, 2),
            "free_pct":                round(free_pct, 2),
            "wall_sharpness_mean_run": round(sharpness, 3),
            "points_per_robot":        clouds,
        }

        with open(self.metrics_file, "a") as f:
            f.write(json.dumps(record) + "\n")

        self.get_logger().info(
            f"[Metrics] t={elapsed:.0f}s | "
            f"unknown={unknown_pct:.1f}% | "
            f"occupied={occupied_pct:.1f}% | "
            f"free={free_pct:.1f}% | "
            f"sharpness={sharpness:.2f} | "
            f"pts={clouds}")


# ─────────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    server = CentralServer()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.log_metrics()
        server.get_logger().info(
            f"[Metrics] Final snapshot saved to {server.metrics_dir}")
        server.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()