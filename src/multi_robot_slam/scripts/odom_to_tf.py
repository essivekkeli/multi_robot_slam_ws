#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        
        # Declare parameters
        self.declare_parameter('robot_name', 'robot1')
        robot_name = self.get_parameter('robot_name').value
        
        # Subscribe to odometry
        self.create_subscription(
            Odometry,
            f'/{robot_name}/odom',
            self.odom_callback,
            10
        )
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info(f'Publishing TF for {robot_name}')
    
    def odom_callback(self, msg):
        # Create transform from odom message
        t = TransformStamped()
        
        # USE THE ODOMETRY MESSAGE'S TIMESTAMP (not current time!)
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id
        t.child_frame_id = msg.child_frame_id
        
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        
        t.transform.rotation = msg.pose.pose.orientation
        
        # Broadcast transform
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = OdomToTF()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()