import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory('multi_robot_slam')
    
    # Launch Gazebo with robots
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gz_multi_robot.launch.py')
        )
    )
    
    # Launch SLAM system (delayed to let Gazebo start)
    slam_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, 'launch', 'multi_robot_slam.launch.py')
                )
            )
        ]
    )
    
    # Launch Nav2 (delayed to let SLAM initialize)
    nav2_launch = TimerAction(
        period=10.0,  # Wait 10 seconds total (5 more after SLAM)
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, 'launch', 'multi_robot_nav2.launch.py')
                ),
                launch_arguments={
                    'num_robots': '3',
                    'use_sim_time': 'True',
                }.items()
            )
        ]
    )
    
    return LaunchDescription([
        gazebo_launch,
        slam_launch,
        nav2_launch
    ])
