import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():
    pkg_multi_robot = get_package_share_directory('multi_robot_slam')
    
    # World file
    world_file = os.path.join(pkg_multi_robot, 'worlds', 'multi_robot_world.sdf')
    
    # URDF file
    urdf_file = os.path.join(pkg_multi_robot, 'urdf', 'robot.urdf.xacro')
    
    # Robot configurations
    robots_config = [
        {'name': 'robot1', 'x': '0.0', 'y': '0.0', 'z': '0.1', 'yaw': '0.0'},
        {'name': 'robot2', 'x': '2.0', 'y': '0.0', 'z': '0.1', 'yaw': '0.0'},
        {'name': 'robot3', 'x': '0.0', 'y': '2.0', 'z': '0.1', 'yaw': '1.57'},
    ]
    
    # Start Gazebo Harmonic
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )
    
    # Bridge for clock
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )
    
    # Create nodes for each robot
    robot_nodes = []
    
    for robot in robots_config:
        robot_name = robot['name']
        
        # Process URDF/xacro
        robot_desc = xacro.process_file(urdf_file).toxml()
        
        # Robot state publisher
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=robot_name,
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_description': robot_desc,
                'frame_prefix': f'{robot_name}/'
            }]
        )
        robot_nodes.append(robot_state_publisher)
        
        # Spawn robot in Gazebo
        spawn_robot = ExecuteProcess(
            cmd=[
                'gz', 'service',
                '-s', '/world/multi_robot_world/create',
                '--reqtype', 'gz.msgs.EntityFactory',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '1000',
                '--req',
                f'sdf_filename: "{urdf_file}", name: "{robot_name}", pose: {{position: {{x: {robot["x"]}, y: {robot["y"]}, z: {robot["z"]}}}, orientation: {{z: {robot["yaw"]}}}}}'
            ],
            output='screen'
        )
        robot_nodes.append(spawn_robot)
        
        # ROS-Gazebo bridges for each robot
        # Bridge: cmd_vel
        cmd_vel_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                f'/{robot_name}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
            ],
            output='screen'
        )
        robot_nodes.append(cmd_vel_bridge)
        
        # Bridge: odometry
        odom_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                f'/{robot_name}/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'
            ],
            output='screen',
            remappings=[
                (f'/{robot_name}/odom', f'/{robot_name}/odom')
            ]
        )
        robot_nodes.append(odom_bridge)
        
        # Bridge: laser scan
        scan_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                f'/{robot_name}/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'
            ],
            output='screen'
        )
        robot_nodes.append(scan_bridge)
        
        # Bridge: TF
        tf_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                f'/{robot_name}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
            ],
            output='screen'
        )
        robot_nodes.append(tf_bridge)
    
    return LaunchDescription([
        gz_sim,
        clock_bridge,
        *robot_nodes
    ])