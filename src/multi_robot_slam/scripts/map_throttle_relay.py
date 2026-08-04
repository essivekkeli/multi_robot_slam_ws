#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid

class MapThrottleRelay(Node):
    def __init__(self):
        super().__init__('map_throttle_relay')
        self.declare_parameter('min_period_sec', 1.0)
        self.min_period = self.get_parameter('min_period_sec').value
        self.last_pub_time = None

        qos = QoSProfile(depth=1,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL,
                          reliability=ReliabilityPolicy.RELIABLE)

        # Relative topic names on purpose: namespace + remappings (applied in
        # launch) determine the real topic, same pattern as every other node
        # in navigation_launch_patched.py. Do NOT hardcode /robotN/map here.
        self.pub = self.create_publisher(OccupancyGrid, 'map_throttled', qos)
        self.sub = self.create_subscription(OccupancyGrid, 'map', self.cb, qos)

    def cb(self, msg):
        self.get_logger().info('cb() called')
        now = self.get_clock().now()
        if self.last_pub_time is None or \
           (now - self.last_pub_time).nanoseconds / 1e9 >= self.min_period:
            self.pub.publish(msg)
            self.last_pub_time = now
            self.get_logger().info('published to map_throttled')

def main():
    rclpy.init()
    node = MapThrottleRelay()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
