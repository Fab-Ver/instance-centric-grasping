"""
Launch file for the multi-object scene manager.

Starts scene_manager with parameters from scene_manager_params.yaml.
The node spawns target objects + distractors, publishes the latched SceneManifest
and stays alive to serve /icgnet/reset_scene.

Usage:
    ros2 launch icgnet_main scene_manager.launch.py
    ros2 launch icgnet_main scene_manager.launch.py target_class:=can target_count:=2
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('icgnet_main')
    params_file = os.path.join(pkg_share, 'config', 'scene_manager_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('target_class', default_value='can',
                              description='Semantic class to spawn as targets'),
        DeclareLaunchArgument('target_count', default_value='2',
                              description='Number of target-class objects'),
        DeclareLaunchArgument('distractor_count', default_value='-1',
                              description='Distractors from other classes (-1 = random 2-3)'),
        Node(
            package='icgnet_main',
            executable='scene_manager',
            name='scene_manager',
            output='screen',
            parameters=[
                params_file,
                {
                    'target_class': LaunchConfiguration('target_class'),
                    'target_count': LaunchConfiguration('target_count'),
                    'distractor_count': LaunchConfiguration('distractor_count'),
                    'use_sim_time': True,
                },
            ],
        ),
    ])
