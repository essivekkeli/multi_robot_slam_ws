import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_share = get_package_share_directory('multi_robot_slam')
    map_merge_pkg = get_package_share_directory('multirobot_map_merge')
    explore_pkg = get_package_share_directory('explore_lite')
    explore_launch_file = os.path.join(explore_pkg, 'launch', 'explore.launch.py')

    # 1. Gazebo + robots
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gz_multi_robot.launch.py')
        )
    )

    # 2. SLAM per robot
    slam_launch = TimerAction(period=5.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'multi_robot_slam.launch.py')
            )
        )
    ])

    # 3. Nav2 per robot
    nav2_launch = TimerAction(period=20.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'multi_robot_nav2.launch.py')
            ),
            launch_arguments={
                'num_robots': '3',
                'use_sim_time': 'True',
            }.items()
        )
    ])

    # 4. Map merge
    map_merge_launch = TimerAction(period=30.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(map_merge_pkg, 'launch', 'map_merge.launch.py')
            )
        )
    ])

    # 5. Frontier exploration — one per robot using namespace argument
    explore_robot1 = TimerAction(period=38.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(explore_launch_file),
            launch_arguments={
                'namespace': 'robot1',
                'use_sim_time': 'true',
                'robot_base_frame': 'robot1/base_footprint',
            }.items()
        )
    ])

    explore_robot2 = TimerAction(period=38.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(explore_launch_file),
            launch_arguments={
                'namespace': 'robot2',
                'use_sim_time': 'true',
                'robot_base_frame': 'robot2/base_footprint',
            }.items()
        )
    ])

    explore_robot3 = TimerAction(period=38.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(explore_launch_file),
            launch_arguments={
                'namespace': 'robot3',
                'use_sim_time': 'true',
                'robot_base_frame': 'robot3/base_footprint',
            }.items()
        )
    ])

    return LaunchDescription([
        gazebo_launch,
        slam_launch,
        nav2_launch,
        map_merge_launch,
        explore_robot1,
        explore_robot2,
        explore_robot3,
    ])