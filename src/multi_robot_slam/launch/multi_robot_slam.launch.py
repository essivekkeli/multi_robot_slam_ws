import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory('multi_robot_slam')
    
    # Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Configuration files
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_params.yaml')
    multi_robot_params_file = os.path.join(pkg_share, 'config', 'multi_robot_params.yaml')
    
    # Robot names
    robot_names = ['robot1', 'robot2', 'robot3']
    
    # Create SLAM nodes for each robot
    slam_nodes = []
    for name in robot_names:  # Changed from robot_name to name
        # Create frame names immediately as strings
        odom_frame = name + '/odom'
        map_frame = name + '/map'
        base_frame = name + '/base_footprint'
        scan_topic = '/' + name + '/scan'
        map_topic = '/' + name + '/map'
        odom_topic = '/' + name + '/odom'
        
        slam_node = Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace=name,
            parameters=[
                slam_params_file,
                {
                    'use_sim_time': use_sim_time,
                    'odom_frame': odom_frame,
                    'map_frame': map_frame,
                    'base_frame': base_frame,
                }
            ],
            remappings=[
                ('scan', scan_topic),
                ('map', map_topic),
                ('odom', odom_topic)
            ],
            output='screen'
        )
        slam_nodes.append(slam_node)
    
    # Central server node
    central_server_node = Node(
        package='multi_robot_slam_py',
        executable='central_server',
        name='central_server',
        parameters=[
            multi_robot_params_file,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )
    
    # RViz
    rviz_config = os.path.join(pkg_share, 'config', 'rviz_config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        *slam_nodes,
        central_server_node,
        rviz_node
    ])