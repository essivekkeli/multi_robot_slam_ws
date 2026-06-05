#!/usr/bin/env python3
"""
Subscribe to /global_map, wait for one message, save as .npy and metadata.
Usage: python3 save_final_map.py <method_label> <output_dir>
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
import numpy as np
import json
import os
import sys

class MapSaver(Node):
    def __init__(self, method, out_dir):
        super().__init__('map_saver')
        self.method  = method
        self.out_dir = out_dir
        os.makedirs(out_dir, exist_ok=True)
        qos = QoSProfile(depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.sub = self.create_subscription(
            OccupancyGrid, '/global_map', self.cb, qos)
        self.get_logger().info(f'Waiting for /global_map...')

    def cb(self, msg):
        grid = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)
        out_npy  = os.path.join(self.out_dir, f'global_map_{self.method}.npy')
        out_meta = os.path.join(self.out_dir, f'global_map_{self.method}_meta.json')
        np.save(out_npy, grid)
        meta = {
            'method':     self.method,
            'resolution': msg.info.resolution,
            'width':      msg.info.width,
            'height':     msg.info.height,
            'origin_x':   msg.info.origin.position.x,
            'origin_y':   msg.info.origin.position.y,
        }
        with open(out_meta, 'w') as f:
            json.dump(meta, f, indent=2)
        self.get_logger().info(f'Saved {out_npy}')
        raise SystemExit

def main():
    method  = sys.argv[1] if len(sys.argv) > 1 else 'unknown'
    out_dir = sys.argv[2] if len(sys.argv) > 2 else '/ros2_ws/results2/maps'
    rclpy.init()
    node = MapSaver(method, out_dir)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
