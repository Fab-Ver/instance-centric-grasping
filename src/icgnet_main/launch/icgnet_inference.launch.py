"""
Launch file per l'inferenza ICGNet locale.

Avvia solo il nodo grasp_service_node con parametri da icgnet_params.yaml.
RViz è già in esecuzione da world.launch.py — non viene lanciata una seconda istanza.

Uso:
    ros2 launch icgnet_main icgnet_inference.launch.py

Prerequisiti:
    - ros2 launch icgnet_main world.launch.py (in un altro terminale)
    - Modifica src/icgnet_main/config/icgnet_params.yaml con i path corretti
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
        parameters=[params_file],
    )

    return LaunchDescription([grasp_node])
