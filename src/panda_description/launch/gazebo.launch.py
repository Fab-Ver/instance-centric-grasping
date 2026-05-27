from ament_index_python.packages import get_package_share_directory
import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    pkg_share = launch_ros.substitutions.FindPackageShare(package='panda_description').find('panda_description')
    default_model_path = os.path.join(pkg_share, 'description', 'models')
    default_urdf_path = os.path.join(default_model_path, 'panda', 'panda.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/rviz.rviz')
    world = LaunchConfiguration('world', default='')

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        ),
        launch_arguments={'verbose': 'true'}.items(),
    )
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'verbose': 'true', 'world': world}.items(),
    )
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', LaunchConfiguration('model')]),
        }],
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'panda'],
        output='screen',
    )
    effort_controller_config = os.path.join(
        get_package_share_directory('panda_description'), 'config', 'ros_control.yaml'
    )
    spawn_arm_controller = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['panda_arm_controller', '--param-file', effort_controller_config],
        output='screen',
    )
    spawn_hand_controller = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['panda_hand_controller', '--param-file', effort_controller_config],
        output='screen',
    )
    spawn_joint_state_broadcaster = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-type', 'joint_state_broadcaster/JointStateBroadcaster'],
        output='screen',
    )
    gazebo_model_path = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[default_model_path])
    gazebo_media_path = SetEnvironmentVariable(name='GAZEBO_MEDIA_PATH', value=[default_model_path])

    return launch.LaunchDescription([
        gazebo_model_path,
        gazebo_media_path,
        launch.actions.DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true',
        ),
        launch.actions.DeclareLaunchArgument(
            name='model',
            default_value=default_urdf_path,
            description='Absolute path to robot urdf file',
        ),
        launch.actions.DeclareLaunchArgument(
            name='rvizconfig',
            default_value=default_rviz_config_path,
            description='Absolute path to rviz config file',
        ),
        launch.actions.DeclareLaunchArgument('world', default_value='', description='World file'),
        spawn_joint_state_broadcaster,
        robot_state_publisher_node,
        rviz_node,
        gzclient,
        gzserver,
        TimerAction(period=15.0, actions=[spawn_entity]),
        spawn_arm_controller,
        spawn_hand_controller,
    ])
