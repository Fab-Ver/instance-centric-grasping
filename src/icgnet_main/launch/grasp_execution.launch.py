"""
Launch file for the grasp executor node.

Starts grasp_executor_node with parameters from grasp_executor_params.yaml.
Requires world.launch.py and icgnet_inference.launch.py running in other terminals.

Usage:
    ros2 launch icgnet_main grasp_execution.launch.py

Then trigger a grasp:
    ros2 service call /icgnet/execute_grasp icgnet_msgs/srv/ExecuteGrasp "{target: 'any'}"
    ros2 service call /icgnet/execute_grasp icgnet_msgs/srv/ExecuteGrasp "{target: 'can'}"
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('icgnet_main')
    params_file = os.path.join(pkg_share, 'config', 'grasp_executor_params.yaml')

    executor_node = Node(
        package='icgnet_main',
        executable='grasp_executor',
        name='grasp_executor_node',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}],
    )

    return LaunchDescription([executor_node])
