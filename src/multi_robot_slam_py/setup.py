from setuptools import setup
import os
from glob import glob

package_name = 'multi_robot_slam_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Multi-robot SLAM Python nodes for ROS2 Jazzy',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'central_server = multi_robot_slam_py.central_server:main',
            'central_server_unified = multi_robot_slam_py.central_server_unified:main',
            'coordination_node = multi_robot_slam_py.coordination_node:main',
            'grid_projector_node = multi_robot_slam_py.grid_projector_node:main',
            'world_tf_broadcaster = multi_robot_slam_py.world_tf_broadcaster:main',
            'smooth_circle = scripts.smooth_circle:main',
        ],
    },
)