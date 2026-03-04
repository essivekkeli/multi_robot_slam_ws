from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('multi_robot_slam')
    params_file = os.path.join(pkg_dir, 'config', 'multi_robot_params.yaml')

    return LaunchDescription([
        Node(
            package='multi_robot_slam_py',
            executable='central_server',
            name='central_server',
            parameters=[params_file, {'use_sim_time': True}],
            output='screen'
        )
    ])
