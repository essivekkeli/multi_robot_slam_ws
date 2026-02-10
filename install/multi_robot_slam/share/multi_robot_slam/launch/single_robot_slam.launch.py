import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('multi_robot_slam')
    
    # Launch arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='robot1',
        description='Name of the robot'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Get launch arguments
    robot_name = LaunchConfiguration('robot_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # SLAM Toolbox parameters file
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_params.yaml')
    
    # SLAM Toolbox node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace=robot_name,
        parameters=[
            slam_params_file,
            {
                'use_sim_time': use_sim_time,
            }
        ],
        remappings=[
            ('scan', ['/', robot_name, '/scan_fixed']),
            ('map', ['/', robot_name, '/map']),
            ('odom', ['/', robot_name, '/odom'])
        ],
        output='screen'
    )
    
    return LaunchDescription([
        robot_name_arg,
        use_sim_time_arg,
        slam_toolbox_node
    ])