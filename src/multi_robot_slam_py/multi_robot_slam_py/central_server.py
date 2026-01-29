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
        
        # Parameters
        self.declare_parameter('num_robots', 3)
        self.declare_parameter('robot_namespaces', ['robot1', 'robot2', 'robot3'])
        self.declare_parameter('map_merge_frequency', 1.0)
        
        self.num_robots = self.get_parameter('num_robots').value
        self.robot_namespaces = self.get_parameter('robot_namespaces').value
        self.merge_frequency = self.get_parameter('map_merge_frequency').value
        
        # Data storage
        self.robot_maps = {}
        self.robot_poses = {}
        self.robot_scans = {}
        self.map_lock = threading.Lock()
        
        # Publishers
        self.global_map_pub = self.create_publisher(
            OccupancyGrid,
            '/global_map',
            10
        )
        
        # Subscribers for each robot
        self.setup_robot_subscribers()
        
        # Timer for map merging
        self.merge_timer = self.create_timer(
            1.0 / self.merge_frequency,
            self.merge_maps_callback
        )
        
        # TF
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.get_logger().info(f'Central Server initialized with {self.num_robots} robots')
    
    def setup_robot_subscribers(self):
        """Setup subscribers for all robots"""
        for robot_ns in self.robot_namespaces:
            # Map subscriber
            self.create_subscription(
                OccupancyGrid,
                f'/{robot_ns}/map',
                lambda msg, ns=robot_ns: self.map_callback(msg, ns),
                10
            )
            
            # Odometry subscriber
            self.create_subscription(
                Odometry,
                f'/{robot_ns}/odom',
                lambda msg, ns=robot_ns: self.odom_callback(msg, ns),
                10
            )
            
            # Scan subscriber
            self.create_subscription(
                LaserScan,
                f'/{robot_ns}/scan',
                lambda msg, ns=robot_ns: self.scan_callback(msg, ns),
                10
            )
            
            self.get_logger().info(f'Subscribed to {robot_ns} topics')
    
    def map_callback(self, msg, robot_ns):
        """Receive map from robot"""
        with self.map_lock:
            self.robot_maps[robot_ns] = msg
            self.get_logger().debug(f'Received map from {robot_ns}: {len(msg.data)} cells')
    
    def odom_callback(self, msg, robot_ns):
        """Receive odometry from robot"""
        self.robot_poses[robot_ns] = msg.pose.pose
    
    def scan_callback(self, msg, robot_ns):
        """Receive laser scan from robot"""
        self.robot_scans[robot_ns] = msg
    
    def merge_maps_callback(self):
        """Periodically merge all robot maps"""
        with self.map_lock:
            if len(self.robot_maps) < 1:
                return
            
            self.get_logger().info(f'Merging {len(self.robot_maps)} maps')
            
            # Simple merging: overlay maps
            merged_map = self.simple_map_merge()
            
            if merged_map is not None:
                self.global_map_pub.publish(merged_map)
                self.get_logger().info('Published merged global map')
    
    def simple_map_merge(self):
        """Simple map merging by overlaying"""
        if not self.robot_maps:
            return None
        
        # Use first map as template
        first_map = list(self.robot_maps.values())[0]
        merged = OccupancyGrid()
        merged.header.stamp = self.get_clock().now().to_msg()
        merged.header.frame_id = 'map'
        merged.info = first_map.info
        
        # Initialize with unknown
        width = first_map.info.width
        height = first_map.info.height
        merged_data = np.full(width * height, -1, dtype=np.int8)
        
        # Overlay all maps
        for robot_ns, robot_map in self.robot_maps.items():
            if len(robot_map.data) != len(merged_data):
                self.get_logger().warn(f'Map size mismatch for {robot_ns}')
                continue
                
            data = np.array(robot_map.data, dtype=np.int8)
            
            # Simple overlay: occupied > free > unknown
            for i in range(len(data)):
                if data[i] == 100:  # Occupied
                    merged_data[i] = 100
                elif data[i] == 0 and merged_data[i] != 100:  # Free
                    merged_data[i] = 0
        
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
        rclpy.shutdown()

if __name__ == '__main__':
    main()
