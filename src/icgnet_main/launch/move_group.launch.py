import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_yaml(package_share: str, rel_path: str) -> dict:
    path = os.path.join(package_share, rel_path)
    with open(path) as f:
        return yaml.safe_load(f)


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    pkg_icgnet = get_package_share_directory('icgnet_main')
    pkg_panda  = get_package_share_directory('panda_ros2_gazebo')

    urdf_path = os.path.join(pkg_panda, 'description', 'models', 'panda', 'panda.urdf')
    srdf_path = os.path.join(pkg_icgnet, 'config', 'moveit', 'panda.srdf')

    with open(urdf_path) as f:
        robot_description_content = f.read()
    with open(srdf_path) as f:
        robot_description_semantic_content = f.read()

    kinematics   = _load_yaml(pkg_icgnet, 'config/moveit/kinematics.yaml')
    joint_limits = _load_yaml(pkg_icgnet, 'config/moveit/joint_limits.yaml')
    ompl         = _load_yaml(pkg_icgnet, 'config/moveit/ompl_planning.yaml')
    controllers  = _load_yaml(pkg_icgnet, 'config/moveit/moveit_controllers.yaml')

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            {'robot_description': robot_description_content},
            {'robot_description_semantic': robot_description_semantic_content},
            {'robot_description_kinematics': kinematics},
            {'robot_description_planning': joint_limits},
            ompl,
            controllers,
            {'use_sim_time': use_sim_time},
            {'publish_robot_description_semantic': True},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation clock'),
        move_group_node,
    ])
