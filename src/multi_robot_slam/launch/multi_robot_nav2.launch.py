
import os
import yaml
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch_ros.actions import PushRosNamespace, SetRemap
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _deep_merge(base, override):
    for key, value in override.items():
        if key in base and isinstance(base[key], dict) and isinstance(value, dict):
            _deep_merge(base[key], value)
        else:
            base[key] = value


def create_robot_params_file(robot_name, base_params_file):
    with open(base_params_file, 'r') as f:
        params = yaml.safe_load(f)

    overrides = {
        'bt_navigator': {'ros__parameters': {
            'global_frame': f'{robot_name}/map',
            'robot_base_frame': f'{robot_name}/base_footprint',
            'odom_topic': f'/{robot_name}/odom',
        }},
        'local_costmap': {'local_costmap': {'ros__parameters': {
            'global_frame': f'{robot_name}/odom',
            'robot_base_frame': f'{robot_name}/base_footprint',
        }}},
        'global_costmap': {'global_costmap': {'ros__parameters': {
            'global_frame': f'{robot_name}/map',
            'robot_base_frame': f'{robot_name}/base_footprint',
        }}},
        'behavior_server': {'ros__parameters': {
            'local_frame': f'{robot_name}/odom',
            'global_frame': f'{robot_name}/map',
            'robot_base_frame': f'{robot_name}/base_footprint',
        }},
        'collision_monitor': {'ros__parameters': {
            'base_frame_id': f'{robot_name}/base_footprint',
            'odom_frame_id': f'{robot_name}/odom',
        }},
    }

    for key, value in overrides.items():
        if key in params:
            _deep_merge(params[key], value)
        else:
            params[key] = value

    tmp = tempfile.NamedTemporaryFile(
        mode='w', prefix=f'nav2_params_{robot_name}_',
        suffix='.yaml', delete=False
    )
    yaml.dump(params, tmp)
    tmp.close()
    return tmp.name


def generate_robot_nav2_launch(context, *args, **kwargs):
    num_robots = int(LaunchConfiguration('num_robots').perform(context))
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    base_params_file = LaunchConfiguration('params_file').perform(context)

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_dir = os.path.join(nav2_bringup_dir, 'launch')
    robot_launches = []

    for i in range(1, num_robots + 1):
        robot_name = f'robot{i}'
        robot_params_file = create_robot_params_file(robot_name, base_params_file)

        robot_group = GroupAction([
            SetRemap('/tf', '/tf'),
            SetRemap('/tf_static', '/tf_static'),
            PushRosNamespace(robot_name),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('multi_robot_slam'), 'launch', 'navigation_launch_patched.py')
                ),
                launch_arguments={
                    #'namespace': robot_name,
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'params_file': robot_params_file,
                    'use_lifecycle_mgr': 'true',
                    'map_subscribe_transient_local': 'true',
                    #'use_docking': 'false',
                }.items(),
            ),
        ])
        robot_launches.append(robot_group)

    return robot_launches


def generate_launch_description():
    pkg_dir = get_package_share_directory('multi_robot_slam')
    default_params_file = os.path.join(pkg_dir, 'config', 'nav2', 'nav2_params_base.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('num_robots', default_value='3'),
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        DeclareLaunchArgument('autostart', default_value='True'),
        DeclareLaunchArgument('params_file', default_value=default_params_file),
        OpaqueFunction(function=generate_robot_nav2_launch)
    ])
