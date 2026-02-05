#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanFrameFixer(Node):
    def __init__(self):
        super().__init__('scan_frame_fixer')
        
        # Declare parameters
        self.declare_parameter('robot_name', 'robot1')
        robot_name = self.get_parameter('robot_name').value
        
        # Subscribe to original scan
        self.create_subscription(
            LaserScan,
            f'/{robot_name}/scan',
            self.scan_callback,
            10
        )
        
        # Publish fixed scan
        self.pub = self.create_publisher(
            LaserScan,
            f'/{robot_name}/scan_fixed',
            10
        )
        
        self.target_frame = f'{robot_name}/lidar'
        self.get_logger().info(f'Fixing scan frame_id to {self.target_frame}')
    
    def scan_callback(self, msg):
        # Fix the frame_id
        msg.header.frame_id = self.target_frame
        # Republish
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = ScanFrameFixer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()