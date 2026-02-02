import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def load_robot_config(config_file):
    """Load robot configuration from YAML file"""
    if not os.path.exists(config_file):
        raise FileNotFoundError(f"Config file not found: {config_file}")
    
    with open(config_file, 'r') as file:
        config = yaml.safe_load(file)
    
    # Check if config loaded properly
    if config is None:
        raise ValueError(f"Config file is empty or invalid: {config_file}")
    
    if 'robots' not in config:
        raise ValueError(f"'robots' key not found in config file: {config_file}")
    
    return config['robots']


def create_robot_nodes(robot_config, urdf_file):
    """Helper function to create nodes for a single robot"""
    name = robot_config['name']
    x_pos = str(robot_config['x'])
    y_pos = str(robot_config['y'])
    z_pos = str(robot_config['z'])
    yaw = str(robot_config['yaw'])
    
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
    
    # Combined bridge for data topics
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'{name}_bridge',
        parameters=[{'use_sim_time': True}],
        arguments=[
            # CMD_VEL - Bidirectional
            f'/{name}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            
            # ODOM - From Gazebo to ROS
            f'/{name}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            
            # SCAN - From Gazebo to ROS
            f'/{name}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            
            # TF - From Gazebo to ROS
            f'/{name}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        remappings=[
            # Remap model pose to robot TF topic
            (f'/model/{name}/pose', f'/{name}/tf'),
        ],
        output='screen'
    )
    nodes.append(bridge_node)

    # TF Republisher - relay individual robot TF to global /tf
    tf_republisher = Node(
        package='topic_tools',
        executable='relay',
        name=f'{name}_tf_relay',
        parameters=[{'use_sim_time': True}],
        arguments=[f'/{name}/tf', '/tf'],
        output='screen'
    )
    nodes.append(tf_republisher)
    
    return nodes


def create_world_frame_broadcasters(robots_config):
    """Create static transforms from world frame to each robot's odom frame"""
    broadcasters = []
    
    for robot in robots_config:
        broadcaster = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'world_to_{robot["name"]}_broadcaster',
            arguments=[
                str(robot['x']),      # x position from config
                str(robot['y']),      # y position from config
                '0.0',                # z (keep at 0 for 2D mapping)
                '0.0',                # roll
                '0.0',                # pitch  
                str(robot['yaw']),    # yaw rotation from config
                'world',              # parent frame
                f'{robot["name"]}/odom'  # child frame
            ],
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
        broadcasters.append(broadcaster)
    
    return broadcasters


def generate_launch_description():
    pkg_multi_robot = get_package_share_directory('multi_robot_slam')
    
    # Load files
    world_file = os.path.join(pkg_multi_robot, 'worlds', 'multi_robot_world.sdf')
    urdf_file = os.path.join(pkg_multi_robot, 'urdf', 'robot.urdf.xacro')
    config_file = os.path.join(pkg_multi_robot, 'config', 'robots.yaml')
    
    # Load robot configuration from YAML
    robots_config = load_robot_config(config_file)
    
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )
    
    # Clock bridge - Use gz.msgs for Jazzy/Harmonic
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )
    
    # Create world frame broadcasters for ALL robots
    world_broadcasters = create_world_frame_broadcasters(robots_config)
    
    # Create nodes for all robots
    robot_nodes = []
    for robot in robots_config:
        robot_nodes.extend(create_robot_nodes(robot, urdf_file))
    
    return LaunchDescription([
        gz_sim,
        clock_bridge,
        *world_broadcasters,
        *robot_nodes
    ])