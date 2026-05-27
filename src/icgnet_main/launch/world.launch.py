import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_icgnet_main = get_package_share_directory('icgnet_main')
    pkg_panda_gazebo = get_package_share_directory('panda_description')

    world_path = os.path.join(pkg_icgnet_main, 'worlds', 'icgnet_world.world')
    models_path = os.path.join(pkg_icgnet_main, 'models')

    # Expose local models to Gazebo (needed by spawn_object -file flag)
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[os.environ.get('GAZEBO_MODEL_PATH', ''), ':', models_path]
    )

    return LaunchDescription([
        set_gazebo_model_path,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_panda_gazebo, 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_path, 'use_sim_time': 'true'}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_icgnet_main, 'launch', 'move_group.launch.py')
            ),
        ),
        # TF: camera position in world frame.
        # Position: [0.97, 0, 0.616], RPY yaw=π pitch=1.0 roll=0 → world→camera_link
        # Must match the sensor pose in icgnet_world.world (mismatch = cloud underground).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.97', '0', '0.616', '3.14159', '1.0', '0', 'world', 'camera_link'],
            parameters=[{'use_sim_time': True}],
        ),
        # TF: ROS optical frame convention
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'camera_link', 'camera_link_optical'],
            parameters=[{'use_sim_time': True}],
        ),
        # TABLE DISABLED — uncomment to re-enable with the table in icgnet_world.world
        # Node(
        #     package='icgnet_main',
        #     executable='static_collision_publisher',
        #     name='static_collision_publisher',
        # ),
        # Object spawn disabled — add objects manually after launch:
        #   ros2 run icgnet_main spawn_object --ros-args -p target_class:=can
        #   ros2 run icgnet_main spawn_object --ros-args -p target_class:=bottle -p num_objects:=3
        # To delete: ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'target_obj'}"
    ])
