#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanFrameFixer(Node):
    def __init__(self):
        super().__init__('scan_frame_fixer')
        self.declare_parameter('robot_name', 'robot1')
        robot_name = self.get_parameter('robot_name').value

        self.create_subscription(
            LaserScan,
            f'/{robot_name}/scan',
            self.scan_callback,
            10
        )
        self.pub = self.create_publisher(LaserScan, f'/{robot_name}/scan_fixed', 10)
        self.target_frame = f'{robot_name}/lidar'
        self.get_logger().info(f'Fixing scan frame_id to {self.target_frame}')

    def scan_callback(self, msg):
        fixed = LaserScan()
        fixed.header.stamp = msg.header.stamp
        fixed.header.frame_id = self.target_frame
        fixed.angle_min = msg.angle_min
        fixed.angle_max = msg.angle_max
        fixed.angle_increment = msg.angle_increment
        fixed.time_increment = msg.time_increment
        fixed.scan_time = msg.scan_time
        fixed.range_min = msg.range_min
        fixed.range_max = msg.range_max
        fixed.ranges = list(msg.ranges)
        fixed.intensities = list(msg.intensities)
        self.pub.publish(fixed)

def main():
    rclpy.init()
    node = ScanFrameFixer()
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

if __name__ == '__main__':
    main()
