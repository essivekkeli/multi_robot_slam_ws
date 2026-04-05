#!/usr/bin/env python3
"""Publishes static transforms from world to each robot's map frame."""
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import yaml, os

class WorldTFBroadcaster(Node):
    def __init__(self):
        super().__init__('world_tf_broadcaster')
        self.broadcaster = StaticTransformBroadcaster(self)
        
        from ament_index_python.packages import get_package_share_directory
        config_path = os.path.join(
            get_package_share_directory('multi_robot_slam'), 'config', 'robots.yaml')
        with open(config_path) as f:
            config = yaml.safe_load(f)
        
        transforms = []
        for robot in config['robots']:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'world'
            t.child_frame_id = f"{robot['name']}/map"
            t.transform.translation.x = float(robot['x'])
            t.transform.translation.y = float(robot['y'])
            t.transform.translation.z = 0.0
            t.transform.rotation.w = 1.0
            transforms.append(t)
            self.get_logger().info(f"Publishing world -> {robot['name']}/map at ({robot['x']}, {robot['y']})")
        
        self.broadcaster.sendTransform(transforms)

def main():
    rclpy.init()
    node = WorldTFBroadcaster()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
