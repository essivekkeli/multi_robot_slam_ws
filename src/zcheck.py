
import rclpy, struct, numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

rclpy.init()
node = Node('zcheck')

def cb(msg):
    n = msg.width * msg.height
    step = msg.point_step
    data = msg.data
    zs = [struct.unpack_from('f', data, i*step + 8)[0] for i in range(min(n, 100))]
    zs = [z for z in zs if abs(z) < 100]
    print(f'z range: min={min(zs):.3f} max={max(zs):.3f} mean={np.mean(zs):.3f}')
    print(f'points with z 0.1-1.5: {sum(0.1 <= z <= 1.5 for z in zs)}')
    rclpy.shutdown()

node.create_subscription(PointCloud2, '/robot1/glim/aligned_points', cb, 10)
rclpy.spin(node)
