#!/usr/bin/env python3
"""
Central Server — Baseline Map Fusion (Stage 3a, Method 1)
Converts each robot GLIM 3D PointCloud2 map to 2D OccupancyGrid
by projecting points within a height band, then merges all robot maps.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
import numpy as np
import struct
import threading


def pointcloud2_to_xyz(msg):
    points = []
    point_step = msg.point_step
    data = msg.data
    offsets = {}
    for field in msg.fields:
        if field.name in ("x", "y", "z"):
            offsets[field.name] = field.offset
    if not all(k in offsets for k in ("x", "y", "z")):
        return np.array([])
    n_points = msg.width * msg.height
    xyz = np.zeros((n_points, 3), dtype=np.float32)
    for i in range(n_points):
        base = i * point_step
        xyz[i, 0] = struct.unpack_from("f", data, base + offsets["x"])[0]
        xyz[i, 1] = struct.unpack_from("f", data, base + offsets["y"])[0]
        xyz[i, 2] = struct.unpack_from("f", data, base + offsets["z"])[0]
    valid = np.isfinite(xyz).all(axis=1)
    return xyz[valid]


def pointcloud_to_occupancy_grid(xyz, resolution=0.05, z_min=0.1, z_max=1.5, frame_id="map"):
    if len(xyz) == 0:
        return None
    mask = np.ones(len(xyz), dtype=bool)
    pts2d = xyz[mask, :2]
    if len(pts2d) == 0:
        return None
    min_x = pts2d[:, 0].min() - 0.5
    min_y = pts2d[:, 1].min() - 0.5
    max_x = pts2d[:, 0].max() + 0.5
    max_y = pts2d[:, 1].max() + 0.5
    width  = int(np.ceil((max_x - min_x) / resolution))
    height = int(np.ceil((max_y - min_y) / resolution))
    grid = np.full(width * height, -1, dtype=np.int8)
    floor_mask = xyz[:, 2] < z_min
    floor_pts = xyz[floor_mask, :2]
    for pt in floor_pts:
        gx = int((pt[0] - min_x) / resolution)
        gy = int((pt[1] - min_y) / resolution)
        if 0 <= gx < width and 0 <= gy < height:
            idx = gy * width + gx
            if grid[idx] != 100:
                grid[idx] = 0
    for pt in pts2d:
        gx = int((pt[0] - min_x) / resolution)
        gy = int((pt[1] - min_y) / resolution)
        if 0 <= gx < width and 0 <= gy < height:
            grid[gy * width + gx] = 100
    msg = OccupancyGrid()
    msg.info.resolution = resolution
    msg.info.width = width
    msg.info.height = height
    msg.info.origin.position.x = min_x
    msg.info.origin.position.y = min_y
    msg.info.origin.orientation.w = 1.0
    msg.header.frame_id = frame_id
    msg.data = grid.tolist()
    return msg


class CentralServer(Node):
    def __init__(self):
        super().__init__("central_server")
        self.declare_parameter("robot_namespaces", ["robot1", "robot2", "robot3"])
        self.declare_parameter("map_merge_frequency", 1.0)
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("z_min", -0.3)
        self.declare_parameter("z_max", 0.3)
        self.robot_namespaces = self.get_parameter("robot_namespaces").value
        self.merge_frequency  = self.get_parameter("map_merge_frequency").value
        self.resolution       = self.get_parameter("resolution").value
        self.z_min            = self.get_parameter("z_min").value
        self.z_max            = self.get_parameter("z_max").value
        self.robot_maps = {}
        self.lock = threading.Lock()
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.global_map_pub = self.create_publisher(OccupancyGrid, "/global_map", qos)
        self.robot_map_pubs = {}
        for name in self.robot_namespaces:
            self.robot_map_pubs[name] = self.create_publisher(OccupancyGrid, f"/{name}/map", qos)
        for name in self.robot_namespaces:
            self.create_subscription(PointCloud2, f"/{name}/glim/aligned_points",
                lambda msg, n=name: self.glim_map_callback(msg, n), 10)
            self.get_logger().info(f"Subscribed to /{name}/glim/aligned_points")
        self.merge_timer = self.create_timer(1.0 / self.merge_frequency, self.merge_and_publish)
        self.get_logger().info("Central Server (baseline fusion) started")

    def glim_map_callback(self, msg, robot_ns):
        xyz = pointcloud2_to_xyz(msg)
        if len(xyz) == 0:
            return
        occ = pointcloud_to_occupancy_grid(xyz, resolution=self.resolution,
            z_min=self.z_min, z_max=self.z_max, frame_id=f"{robot_ns}/map")
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
        merged = np.full(width * height, -1, dtype=np.int8)
        for robot_ns, m in maps.items():
            ox = m.info.origin.position.x
            oy = m.info.origin.position.y
            x_offset = int(round((ox - min_x) / resolution))
            y_offset = int(round((oy - min_y) / resolution))
            data = np.array(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)
            for row in range(m.info.height):
                for col in range(m.info.width):
                    gx = x_offset + col
                    gy = y_offset + row
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
