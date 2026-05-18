import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_icgnet_main = get_package_share_directory('icgnet_main')
    pkg_panda_gazebo = get_package_share_directory('panda_ros2_gazebo')

    target_type = LaunchConfiguration('target_type', default='coke_can')
    num_objects = LaunchConfiguration('num_objects', default='1')
    mode = LaunchConfiguration('mode', default='offline')

    world_path = os.path.join(pkg_icgnet_main, 'worlds', 'icgnet_world.world')
    models_path = os.path.join(pkg_icgnet_main, 'models')

    # Append local models without overwriting standard Gazebo model database
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[os.environ.get('GAZEBO_MODEL_PATH', ''), ':', models_path]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'target_type',
            default_value='cylinder_offline',
            description='Target object type'
        ),
        DeclareLaunchArgument(
            'num_objects',
            default_value='1',
            description='Total number of objects to spawn (1-5)'
        ),
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
        # TF: camera position in world frame.
        # Position chosen so the camera is ~0.65m from the workspace center [0.65,0,0.05]
        # at ~30° elevation from vertical — within ICGNet training range r∈[0.48,0.72]m, θ∈[0°,45°].
        # RPY is yaw=π, pitch=1.0, roll=0 (arg order: x y z yaw pitch roll parent child).
        # The optical-frame z-axis in world is [-0.54, 0, -0.84], looking at ~[0.61, 0, 0.05].
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.97', '0', '0.616', '3.14159', '1.0', '0', 'world', 'camera_link'],
        ),
        # TF: ROS optical frame convention
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'camera_link', 'camera_link_optical'],
        ),
        # Spawn objects at random positions
        Node(
            package='icgnet_main',
            executable='spawn_object',
            parameters=[{
                'target_type': target_type,
                'num_objects': num_objects
            }],
            output='screen',
            emulate_tty=True
        )
    ])
