"""
Launch ICGNet local inference node.

Starts grasp_service_node with parameters from icgnet_params.yaml.
RViz is already running from world.launch.py — no second instance is launched.

Usage:
    ros2 launch icgnet_main icgnet_inference.launch.py

Prerequisites:
    - ros2 launch icgnet_main world.launch.py (in another terminal)
    - Edit src/icgnet_main/config/icgnet_params.yaml with correct paths
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('icgnet_main')
    params_file = os.path.join(pkg_share, 'config', 'icgnet_params.yaml')

    grasp_node = Node(
        package='icgnet_main',
        executable='grasp_service_node',
        name='icgnet_grasp_node',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}],
    )

    return LaunchDescription([grasp_node])
