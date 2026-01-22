import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/essivekkeli/multi_robot_slam_ws/install/multi_robot_slam_py'
