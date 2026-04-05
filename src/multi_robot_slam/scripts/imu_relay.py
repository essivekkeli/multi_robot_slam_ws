#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu

class ImuRelay(Node):
    def __init__(self):
        super().__init__('imu_relay')
        self.declare_parameter('robot_name', 'robot1')
        name = self.get_parameter('robot_name').value
        
        reliable_qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE)
        best_effort_qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE)
        
        self.pub = self.create_publisher(Imu, f'/{name}/imu/best_effort', best_effort_qos)
        self.sub = self.create_subscription(Imu, f'/{name}/imu', self.cb, reliable_qos)
        self.get_logger().info(f'IMU relay started for {name}')

    def cb(self, msg):
        self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(ImuRelay())

if __name__ == '__main__':
    main()
