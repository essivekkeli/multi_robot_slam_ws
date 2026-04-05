#!/usr/bin/env python3
"""
Central Server — Stage 3a Week 1: TF-based world frame fusion
Accumulates GLIM aligned_points per robot, transforms to world frame
using TF, then projects accumulated 3D cloud to 2D OccupancyGrid.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import numpy as np
import struct
import threading


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


def accumulated_cloud_to_occupancy_grid(xyz, resolution=0.05, z_min=0.1, z_max=2.5, frame_id="world"):
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


class CentralServer(Node):
    def __init__(self):
        super().__init__("central_server")
        self.declare_parameter("robot_namespaces", ["robot1", "robot2", "robot3"])
        self.declare_parameter("map_merge_frequency", 1.0)
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("z_min", 0.1)
        self.declare_parameter("z_max", 2.5)
        self.declare_parameter("max_points_per_robot", 500000)

        self.robot_namespaces = self.get_parameter("robot_namespaces").value
        self.merge_frequency  = self.get_parameter("map_merge_frequency").value
        self.resolution       = self.get_parameter("resolution").value
        self.z_min            = self.get_parameter("z_min").value
        self.z_max            = self.get_parameter("z_max").value
        self.max_points       = self.get_parameter("max_points_per_robot").value

        # TF buffer and listener
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Accumulated point clouds in world frame per robot
        self.robot_clouds = {name: np.zeros((0, 3), dtype=np.float32)
                             for name in self.robot_namespaces}
        self.robot_maps = {}
        self.global_cloud = np.zeros((0, 3), dtype=np.float32)
        self.lock = threading.Lock()

        qos = QoSProfile(depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)

        self.global_map_pub = self.create_publisher(OccupancyGrid, "/global_map", qos)
        self.robot_map_pubs = {}
        for name in self.robot_namespaces:
            self.robot_map_pubs[name] = self.create_publisher(
                OccupancyGrid, f"/{name}/map", qos)

        for name in self.robot_namespaces:
            self.create_subscription(
                PointCloud2, f"/{name}/glim/aligned_points",
                lambda msg, n=name: self.pointcloud_callback(msg, n), 10)
            self.get_logger().info(f"Subscribed to /{name}/glim/aligned_points")

        self.merge_timer = self.create_timer(
            1.0 / self.merge_frequency, self.merge_and_publish)
        self.get_logger().info("Central Server (TF-based world frame fusion) started")

    def pointcloud_callback(self, msg, robot_ns):
        # Transform point cloud to world frame using TF
        try:
            transform = self.tf_buffer.lookup_transform(
                'world',
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))
            msg_world = do_transform_cloud(msg, transform)
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            # Fall back to robot map frame if world TF not available
            self.get_logger().warn(f"TF lookup failed for {robot_ns}: {e}", throttle_duration_sec=5.0)
            msg_world = msg

        xyz = pointcloud2_to_xyz(msg_world)
        if len(xyz) == 0:
            return

        with self.lock:
            current = self.robot_clouds[robot_ns]
            combined = np.vstack([current, xyz]) if len(current) > 0 else xyz
            if len(combined) > self.max_points:
                combined = _voxel_downsample(combined, voxel_size=0.05)
            self.robot_clouds[robot_ns] = combined

        # Per-robot map in world frame
        occ = accumulated_cloud_to_occupancy_grid(
            combined,
            resolution=self.resolution,
            z_min=self.z_min,
            z_max=self.z_max,
            frame_id='world')

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
            ox = m.info.origin.position.x
            oy = m.info.origin.position.y
            x_off = int(round((ox - min_x) / resolution))
            y_off = int(round((oy - min_y) / resolution))
            data = np.array(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)
            for row in range(m.info.height):
                for col in range(m.info.width):
                    gx = x_off + col
                    gy = y_off + row
                    if 0 <= gx < width and 0 <= gy < height:
                        cell = data[row, col]
                        idx = gy * width + gx
                        if cell == 100:
                            merged[idx] = 100
                        elif cell == 0 and merged[idx] != 100:
                            merged[idx] = 0

        global_map = OccupancyGrid()
        global_map.header.stamp = self.get_clock().now().to_msg()
        global_map.header.frame_id = "world"
        global_map.info.resolution = resolution
        global_map.info.width = width
        global_map.info.height = height
        global_map.info.origin.position.x = min_x
        global_map.info.origin.position.y = min_y
        global_map.info.origin.orientation.w = 1.0
        global_map.data = merged.tolist()
        self.global_map_pub.publish(global_map)


def main(args=None):
    rclpy.init(args=args)
    server = CentralServer()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
