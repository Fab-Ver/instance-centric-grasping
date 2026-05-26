"""
Launch ICGNet replay node (no GPU required).

Loads saved inference data (grasps + collision objects) from a directory
produced by save_inference_node and replays them on /icgnet/compute_grasps trigger.
Drop-in replacement for icgnet_inference.launch.py for teammates without a GPU.

Usage:
    ros2 launch icgnet_main icgnet_replay.launch.py \\
        inference_dir:=/path/to/saved/data

Then trigger exactly as with the real inference node:
    ros2 service call /icgnet/compute_grasps std_srvs/srv/Trigger

Prerequisites:
    - ros2 launch icgnet_main world.launch.py (Gazebo + MoveIt2 running)
    - Saved inference directory with grasps.bin, collision_*.bin, meta.json
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    inference_dir_arg = DeclareLaunchArgument(
        'inference_dir',
        default_value=os.path.expanduser('~/icgnet_inference_data'),
        description='Directory containing saved ICGNet inference data.',
    )
    spawn_object_arg = DeclareLaunchArgument(
        'spawn_object',
        default_value='true',
        description='Spawn the saved object in Gazebo on each trigger (requires object_sdf_path in meta.json).',
    )

    replay_node = Node(
        package='icgnet_main',
        executable='replay_inference_node',
        name='replay_inference_node',
        output='screen',
        parameters=[{
            'inference_dir': LaunchConfiguration('inference_dir'),
            'spawn_object': LaunchConfiguration('spawn_object'),
            'use_sim_time': True,
        }],
    )

    return LaunchDescription([
        inference_dir_arg,
        spawn_object_arg,
        replay_node,
    ])
