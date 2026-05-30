import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='')

    pkg_panda = get_package_share_directory('panda_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    urdf_path = os.path.join(
        pkg_panda, 'description', 'models', 'panda', 'panda.urdf'
    )
    with open(urdf_path, 'r') as f:
        robot_description_content = f.read()
    # gz_ros2_control and MoveIt do not expand $(find ...) — substitute at launch time.
    robot_description_content = robot_description_content.replace(
        '$(find panda_description)', pkg_panda
    )

    rviz_config = os.path.join(pkg_panda, 'rviz', 'rviz.rviz')

    # Gz-sim: server + GUI with OGRE1 renderer (WSL2 compatible).
    # Removing -s runs server+GUI together. render_engine ogre avoids OGRE2 crash on WSL2.
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [world, ' -r --render-engine ogre'],
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content,
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    # Spawn the robot into gz-sim using the robot_description topic.
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'icgnet_world',
            '-topic', 'robot_description',
            '-name', 'panda',
        ],
        output='screen',
    )

    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    spawn_arm = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['panda_arm_controller'],
        output='screen',
    )
    spawn_hand = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['panda_hand_controller'],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value=''),
        gz_sim,
        robot_state_publisher,
        rviz,
        # Spawn robot after gz-sim has started (3s).
        TimerAction(period=3.0, actions=[spawn_robot]),
        # Controllers after robot is spawned (6s total).
        TimerAction(period=6.0, actions=[spawn_jsb]),
        TimerAction(period=8.0, actions=[spawn_arm]),
        TimerAction(period=8.0, actions=[spawn_hand]),
    ])
