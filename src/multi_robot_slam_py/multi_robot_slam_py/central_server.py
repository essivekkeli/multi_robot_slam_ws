#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import numpy as np
from collections import defaultdict
import threading

class CentralServer(Node):
    def __init__(self):
        super().__init__('central_server')
        
        self.declare_parameter('num_robots', 3)
        self.declare_parameter('robot_namespaces', ['robot1', 'robot2', 'robot3'])
        self.declare_parameter('map_merge_frequency', 1.0)
        
        self.num_robots = self.get_parameter('num_robots').value
        self.robot_namespaces = self.get_parameter('robot_namespaces').value
        self.merge_frequency = self.get_parameter('map_merge_frequency').value
        
        self.robot_maps = {}
        self.robot_poses = {}
        self.robot_scans = {}
        self.map_lock = threading.Lock()
        
        self.global_map_pub = self.create_publisher(OccupancyGrid, '/global_map', 10)
        
        self.setup_robot_subscribers()
        
        self.merge_timer = self.create_timer(
            1.0 / self.merge_frequency,
            self.merge_maps_callback
        )
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.get_logger().info(f'Central Server initialized with {self.num_robots} robots')
    
    def setup_robot_subscribers(self):
        for robot_ns in self.robot_namespaces:
            self.create_subscription(OccupancyGrid, f'/{robot_ns}/map',
                lambda msg, ns=robot_ns: self.map_callback(msg, ns), 10)
            self.create_subscription(Odometry, f'/{robot_ns}/odom',
                lambda msg, ns=robot_ns: self.odom_callback(msg, ns), 10)
            self.create_subscription(LaserScan, f'/{robot_ns}/scan',
                lambda msg, ns=robot_ns: self.scan_callback(msg, ns), 10)
            self.get_logger().info(f'Subscribed to {robot_ns} topics')
    
    def map_callback(self, msg, robot_ns):
        with self.map_lock:
            self.robot_maps[robot_ns] = msg
    
    def odom_callback(self, msg, robot_ns):
        self.robot_poses[robot_ns] = msg.pose.pose
    
    def scan_callback(self, msg, robot_ns):
        self.robot_scans[robot_ns] = msg
    
    def merge_maps_callback(self):
        with self.map_lock:
            if len(self.robot_maps) < 1:
                return
            merged_map = self.simple_map_merge()
            if merged_map is not None:
                self.global_map_pub.publish(merged_map)
    
    def simple_map_merge(self):
        if not self.robot_maps:
            return None

        resolution = None
        min_x = float('inf')
        min_y = float('inf')
        max_x = float('-inf')
        max_y = float('-inf')

        for robot_map in self.robot_maps.values():
            info = robot_map.info
            if resolution is None:
                resolution = info.resolution
            ox = info.origin.position.x
            oy = info.origin.position.y
            min_x = min(min_x, ox)
            min_y = min(min_y, oy)
            max_x = max(max_x, ox + info.width * info.resolution)
            max_y = max(max_y, oy + info.height * info.resolution)

        width  = int(np.ceil((max_x - min_x) / resolution))
        height = int(np.ceil((max_y - min_y) / resolution))
        merged_data = np.full(width * height, -1, dtype=np.int8)

        for robot_ns, robot_map in self.robot_maps.items():
            info = robot_map.info
            ox = info.origin.position.x
            oy = info.origin.position.y
            data = np.array(robot_map.data, dtype=np.int8).reshape(info.height, info.width)

            x_offset = int(round((ox - min_x) / resolution))
            y_offset = int(round((oy - min_y) / resolution))

            for row in range(info.height):
                for col in range(info.width):
                    gx = x_offset + col
                    gy = y_offset + row
                    if 0 <= gx < width and 0 <= gy < height:
                        cell = data[row, col]
                        idx = gy * width + gx
                        if cell == 100:
                            merged_data[idx] = 100
                        elif cell == 0 and merged_data[idx] != 100:
                            merged_data[idx] = 0

        merged = OccupancyGrid()
        merged.header.stamp = self.get_clock().now().to_msg()
        merged.header.frame_id = 'world'
        merged.info.resolution = resolution
        merged.info.width = width
        merged.info.height = height
        merged.info.origin.position.x = min_x
        merged.info.origin.position.y = min_y
        merged.info.origin.orientation.w = 1.0
        merged.data = merged_data.tolist()
        return merged


def main(args=None):
    rclpy.init(args=args)
    central_server = CentralServer()
    try:
        rclpy.spin(central_server)
    except KeyboardInterrupt:
        pass
    finally:
        central_server.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
