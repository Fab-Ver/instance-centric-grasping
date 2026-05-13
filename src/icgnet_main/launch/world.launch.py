import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_icgnet_main = get_package_share_directory('icgnet_main')
    pkg_panda_gazebo = get_package_share_directory('panda_ros2_gazebo')

    # Percorso del tuo file .world
    world_path = os.path.join(pkg_icgnet_main, 'worlds', 'icgnet_table.world')
    
    # Percorso dei modelli locali
    models_path = os.path.join(pkg_icgnet_main, 'models')
    
    # Aggiungiamo i modelli locali al path di Gazebo
    # Nota: usiamo append per non cancellare i modelli standard di Gazebo
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[os.environ.get('GAZEBO_MODEL_PATH', ''), ':', models_path]
    )

    return LaunchDescription([
        set_gazebo_model_path,
        # Gazebo + Panda + RViz + controllers
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_panda_gazebo, 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_path}.items(),
        ),
        # MoveIt2 move_group node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_icgnet_main, 'launch', 'move_group.launch.py')
            ),
        ),
        # TF: camera position in world frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['1.3', '0', '1.2', '3.14159', '0.8', '0', 'world', 'camera_link'],
        ),
        # TF: ROS optical frame convention
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'camera_link', 'camera_link_optical'],
        )
    ])
