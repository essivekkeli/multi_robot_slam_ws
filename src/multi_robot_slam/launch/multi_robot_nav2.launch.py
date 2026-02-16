#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace


def generate_robot_nav2_launch(context, *args, **kwargs):
    num_robots = int(LaunchConfiguration('num_robots').perform(context))
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_dir = os.path.join(nav2_bringup_dir, 'launch')
    
    robot_launches = []
    
    for i in range(1, num_robots + 1):
        robot_name = f'robot{i}'
        
        robot_group = GroupAction([
            PushRosNamespace(robot_name),
            
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_launch_dir, 'navigation_launch.py')
                ),
                launch_arguments={
                    'namespace': robot_name,
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'params_file': params_file,
                    'use_lifecycle_mgr': 'false',
                    'map_subscribe_transient_local': 'true',
                    'use_docking': 'false',  # Disable docking server
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
