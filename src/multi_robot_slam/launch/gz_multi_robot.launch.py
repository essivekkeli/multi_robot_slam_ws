import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def create_robot_nodes(robot_config, urdf_file):
    """Helper function to create nodes for a single robot"""
    name = robot_config['name']
    x_pos = robot_config['x']
    y_pos = robot_config['y']
    z_pos = robot_config['z']
    yaw = robot_config['yaw']
    
    nodes = []
    
    # Process URDF
    robot_desc = xacro.process_file(
        urdf_file,
        mappings={'robot_name': name}
    ).toxml()
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=name,
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_desc,
            'frame_prefix': name + '/'
        }]
    )
    nodes.append(robot_state_publisher)
    
    # Spawn Robot
    spawn_robot = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'ros_gz_sim', 'create',
                    '--ros-args',
                    '-r', '__ns:=/',
                    '--',
                    '-name', name,
                    '-topic', '/' + name + '/robot_description',
                    '-world', 'multi_robot_world',
                    '-x', x_pos,
                    '-y', y_pos,
                    '-z', z_pos,
                    '-Y', yaw,
                ],
                output='screen'
            )
        ]
    )
    nodes.append(spawn_robot)
    
    # CMD_VEL Bridge
    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/' + name + '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        output='screen'
    )
    nodes.append(cmd_vel_bridge)
    
    # Odometry Bridge
    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/' + name + '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry'],  # Changed ] to [
        output='screen'
    )
    nodes.append(odom_bridge)
    
   # Scan Bridge 
    scan_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/' + name + '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'],  # Changed ] to [
        output='screen'
    )
    nodes.append(scan_bridge)
    
    # TF Bridge
    tf_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/' + name + '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'],  
        output='screen'
    )
    nodes.append(tf_bridge)
    
    return nodes

def generate_launch_description():
    pkg_multi_robot = get_package_share_directory('multi_robot_slam')
    world_file = os.path.join(pkg_multi_robot, 'worlds', 'multi_robot_world.sdf')
    urdf_file = os.path.join(pkg_multi_robot, 'urdf', 'robot.urdf.xacro')
    
    robots_config = [
        {'name': 'robot1', 'x': '0.0', 'y': '0.0', 'z': '0.1', 'yaw': '0.0'},
        {'name': 'robot2', 'x': '2.0', 'y': '0.0', 'z': '0.1', 'yaw': '0.0'},
        {'name': 'robot3', 'x': '0.0', 'y': '2.0', 'z': '0.1', 'yaw': '1.57'},
    ]
    
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )
    
    # Clock bridge
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],  # Changed ] to [
        output='screen'
    )
    
    # Create nodes for all robots using helper function
    robot_nodes = []
    for robot in robots_config:
        robot_nodes.extend(create_robot_nodes(robot, urdf_file))
    
    return LaunchDescription([
        gz_sim,
        clock_bridge,
        *robot_nodes
    ])