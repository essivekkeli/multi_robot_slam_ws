import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

rclpy.init()
node = Node('test_sub')

def cb(msg):
    print('frame:', msg.header.frame_id)
    print('height:', msg.height, 'width:', msg.width)
    print('point_step:', msg.point_step)
    print('fields:', [f.name for f in msg.fields])
    rclpy.shutdown()

node.create_subscription(PointCloud2, '/glim_ros/map', cb, 10)
rclpy.spin(node)
