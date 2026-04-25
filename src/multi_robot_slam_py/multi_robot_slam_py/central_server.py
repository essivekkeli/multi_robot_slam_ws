#!/usr/bin/env python3
"""
Central Server — Stage 3a Week 1: TF-based world frame fusion
+ Stage 3b metrics logging for thesis documentation.
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


def pointcloud2_to_xyz(msg):
    offsets = {}
    for field in msg.fields:
        if field.name in ("x", "y", "z"):
            offsets[field.name] = field.offset
    if not all(k in offsets for k in ("x", "y", "z")):
        return np.array([])
    n = msg.width * msg.height
    step = msg.point_step
    data = msg.data
    xyz = np.zeros((n, 3), dtype=np.float32)
    for i in range(n):
        base = i * step
        xyz[i, 0] = struct.unpack_from("f", data, base + offsets["x"])[0]
        xyz[i, 1] = struct.unpack_from("f", data, base + offsets["y"])[0]
        xyz[i, 2] = struct.unpack_from("f", data, base + offsets["z"])[0]
    valid = np.isfinite(xyz).all(axis=1)
    return xyz[valid]


def accumulated_cloud_to_occupancy_grid(xyz, resolution=0.05,
                                        z_min=0.1, z_max=2.5, frame_id="world"):
    if len(xyz) == 0:
        return None
    mask = (xyz[:, 2] >= z_min) & (xyz[:, 2] <= z_max)
    pts2d = xyz[mask, :2]
    if len(pts2d) == 0:
        return None
    floor_mask = (xyz[:, 2] >= -0.5) & (xyz[:, 2] < z_min)
    floor_pts = xyz[floor_mask, :2]
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
        valid = (gx >= 0) & (gx < width) & (gy >= 0) & (gy < height)
        grid[gy[valid], gx[valid]] = 0
    gx = ((pts2d[:, 0] - min_x) / resolution).astype(int)
    gy = ((pts2d[:, 1] - min_y) / resolution).astype(int)
    valid = (gx >= 0) & (gx < width) & (gy >= 0) & (gy < height)
    grid[gy[valid], gx[valid]] = 100
    msg = OccupancyGrid()
    msg.info.resolution = resolution
    msg.info.width = width
    msg.info.height = height
    msg.info.origin.position.x = float(min_x)
    msg.info.origin.position.y = float(min_y)
    msg.info.origin.orientation.w = 1.0
    msg.header.frame_id = frame_id
    msg.data = grid.flatten().tolist()
    return msg


def _voxel_downsample(xyz, voxel_size=0.05):
    voxels = np.floor(xyz / voxel_size).astype(int)
    _, unique_idx = np.unique(voxels, axis=0, return_index=True)
    return xyz[unique_idx]


def _wall_sharpness(grid_np):
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


class CentralServer(Node):
    def __init__(self):
        super().__init__("central_server")
        self.declare_parameter("robot_namespaces", ["robot1", "robot2", "robot3"])
        self.declare_parameter("map_merge_frequency", 1.0)
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("z_min", 0.1)
        self.declare_parameter("z_max", 2.5)
        self.declare_parameter("max_points_per_robot", 500000)
        self.declare_parameter("metrics_interval_sec", 30.0)
        self.declare_parameter("method_label", "baseline_tf")

        self.robot_namespaces  = self.get_parameter("robot_namespaces").value
        self.merge_frequency   = self.get_parameter("map_merge_frequency").value
        self.resolution        = self.get_parameter("resolution").value
        self.z_min             = self.get_parameter("z_min").value
        self.z_max             = self.get_parameter("z_max").value
        self.max_points        = self.get_parameter("max_points_per_robot").value
        self.metrics_interval  = self.get_parameter("metrics_interval_sec").value
        self.method_label      = self.get_parameter("method_label").value

        run_ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.metrics_dir = f"/tmp/fusion_metrics/{self.method_label}_{run_ts}"
        os.makedirs(self.metrics_dir, exist_ok=True)
        self.metrics_file = os.path.join(self.metrics_dir, "metrics.jsonl")
        self.start_time = time.time()

        with open(os.path.join(self.metrics_dir, "run_info.json"), "w") as f:
            json.dump({
                "method": self.method_label,
                "started": run_ts,
                "resolution": self.resolution,
                "z_min": self.z_min,
                "z_max": self.z_max,
                "robots": list(self.robot_namespaces),
            }, f, indent=2)

        self.get_logger().info(f"[Metrics] Logging to {self.metrics_dir}")

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.robot_clouds = {name: np.zeros((0, 3), dtype=np.float32)
                             for name in self.robot_namespaces}
        self.robot_maps        = {}
        self.global_map_latest = None
        self.lock = threading.Lock()

        qos = QoSProfile(depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.global_map_pub = self.create_publisher(OccupancyGrid, "/global_map", qos)
        self.robot_map_pubs = {
            name: self.create_publisher(OccupancyGrid, f"/{name}/map", qos)
            for name in self.robot_namespaces
        }

        for name in self.robot_namespaces:
            self.create_subscription(
                PointCloud2, f"/{name}/glim/aligned_points",
                lambda msg, n=name: self.pointcloud_callback(msg, n), 10)
            self.get_logger().info(f"Subscribed to /{name}/glim/aligned_points")

        self.merge_timer   = self.create_timer(1.0 / self.merge_frequency,
                                               self.merge_and_publish)
        self.metrics_timer = self.create_timer(self.metrics_interval,
                                               self.log_metrics)

        self.get_logger().info(
            f"Central Server started | method={self.method_label} | "
            f"metrics every {self.metrics_interval}s")

    def pointcloud_callback(self, msg, robot_ns):
        try:
            transform = self.tf_buffer.lookup_transform(
                'world', msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))
            msg_world = do_transform_cloud(msg, transform)
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f"TF lookup failed for {robot_ns}: {e}", throttle_duration_sec=5.0)
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
            z_min=self.z_min, z_max=self.z_max, frame_id='world')
        if occ is None:
            return
        occ.header.stamp = msg.header.stamp

        with self.lock:
            self.robot_maps[robot_ns] = occ
        self.robot_map_pubs[robot_ns].publish(occ)

    def merge_and_publish(self):
        with self.lock:
            if not self.robot_maps:
                return
            maps = dict(self.robot_maps)

        resolution = self.resolution
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
        merged = np.full(height * width, -1, dtype=np.int8)

        for m in maps.values():
            ox    = m.info.origin.position.x
            oy    = m.info.origin.position.y
            x_off = int(round((ox - min_x) / resolution))
            y_off = int(round((oy - min_y) / resolution))
            data  = np.array(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)
            rows  = np.arange(m.info.height)
            cols  = np.arange(m.info.width)
            cc, rr = np.meshgrid(cols, rows)
            gx = x_off + cc
            gy = y_off + rr
            valid = (gx >= 0) & (gx < width) & (gy >= 0) & (gy < height)
            idx   = (gy * width + gx)[valid]
            cell  = data[valid]
            occupied_mask = cell == 100
            free_mask     = (cell == 0) & (merged[idx] != 100)
            merged[idx[occupied_mask]] = 100
            merged[idx[free_mask]]     = 0

        global_map = OccupancyGrid()
        global_map.header.stamp            = self.get_clock().now().to_msg()
        global_map.header.frame_id         = "world"
        global_map.info.resolution         = resolution
        global_map.info.width              = width
        global_map.info.height             = height
        global_map.info.origin.position.x  = min_x
        global_map.info.origin.position.y  = min_y
        global_map.info.origin.orientation.w = 1.0
        global_map.data = merged.tolist()
        self.global_map_pub.publish(global_map)

        with self.lock:
            self.global_map_latest = global_map

    def log_metrics(self):
        with self.lock:
            gmap   = self.global_map_latest
            clouds = {n: len(c) for n, c in self.robot_clouds.items()}

        if gmap is None:
            self.get_logger().info("[Metrics] No global map yet, skipping.")
            return

        data = np.array(gmap.data, dtype=np.int8)
        total        = len(data)
        unknown_pct  = float(np.sum(data == -1)  / total * 100)
        occupied_pct = float(np.sum(data == 100) / total * 100)
        free_pct     = float(np.sum(data == 0)   / total * 100)
        grid_2d      = data.reshape(gmap.info.height, gmap.info.width)
        sharpness    = _wall_sharpness(grid_2d)
        elapsed      = time.time() - self.start_time

        record = {
            "timestamp":              datetime.now().isoformat(),
            "elapsed_sec":            round(elapsed, 1),
            "method":                 self.method_label,
            "map_width":              gmap.info.width,
            "map_height":             gmap.info.height,
            "total_cells":            total,
            "unknown_pct":            round(unknown_pct, 2),
            "occupied_pct":           round(occupied_pct, 2),
            "free_pct":               round(free_pct, 2),
            "wall_sharpness_mean_run": round(sharpness, 3),
            "points_per_robot":       clouds,
        }

        with open(self.metrics_file, "a") as f:
            f.write(json.dumps(record) + "\n")

        self.get_logger().info(
            f"[Metrics] t={elapsed:.0f}s | "
            f"unknown={unknown_pct:.1f}% | "
            f"occupied={occupied_pct:.1f}% | "
            f"free={free_pct:.1f}% | "
            f"sharpness={sharpness:.2f} | "
            f"pts={clouds}"
        )


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
