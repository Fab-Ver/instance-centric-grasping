import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_icgnet_main = get_package_share_directory('icgnet_main')
    pkg_panda_gazebo = get_package_share_directory('panda_ros2_gazebo')

    # Percorso del tuo file .world
    world_path = os.path.join(pkg_icgnet_main, 'worlds', 'icgnet_table.world')

    return LaunchDescription([
        # Lanciamo il file del panda che ora accetta il parametro 'world'
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_panda_gazebo, 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_path}.items(),
        ),
        # TF Gazebo Standard (Pose della camera)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['1.3', '0', '1.2', '3.14159', '0.8', '0', 'world', 'camera_link']
        ),
        # TF Ottico ROS (Rotazione 90 gradi per allineare PointCloud)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'camera_link', 'camera_link_optical']
        )
    ])
