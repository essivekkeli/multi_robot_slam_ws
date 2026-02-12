#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class SmoothCircle(Node):
    def __init__(self):
        super().__init__('smooth_circle')
        
        self.declare_parameter('robot_name', 'robot1')
        robot_name = self.get_parameter('robot_name').value
        
        self.pub = self.create_publisher(
            Twist,
            f'/{robot_name}/cmd_vel',
            10
        )
        
        self.timer = self.create_timer(0.1, self.move)
        self.get_logger().info(f'Driving {robot_name} in smooth circle')
    
    def move(self):
        msg = Twist()
        msg.linear.x = 0.2  # Slow forward speed
        msg.angular.z = 0.3  # Gentle turn
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = SmoothCircle()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()