import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_gpu = LaunchConfiguration('use_gpu', default='false')
    headless = LaunchConfiguration('headless', default='true')
    rviz = LaunchConfiguration('rviz', default='true')

    pkg_icgnet_main = get_package_share_directory('icgnet_main')
    pkg_panda = get_package_share_directory('panda_description')

    # gz-sim world SDF (Fortress / DART physics)
    world_path = os.path.join(pkg_icgnet_main, 'worlds', 'icgnet_world.sdf')

    # GZ_SIM_RESOURCE_PATH: exposes local models to gz-sim URI resolution.
    models_path = os.path.join(pkg_icgnet_main, 'models')
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[os.environ.get('GZ_SIM_RESOURCE_PATH', ''), ':', models_path]
    )

    return LaunchDescription([
        set_gz_resource_path,

        DeclareLaunchArgument(
            'headless',
            default_value='true',
            description='Run Gazebo server-only (no GUI window). Camera sensor still works.',
        ),

        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description=(
                'Launch the RViz visualization stack (RViz + scene_visualizer). '
                'Set false for headless automated runs (e.g. run_evaluation_phase1.py) to speed them up.'
            ),
        ),

        DeclareLaunchArgument(
            'use_gpu',
            default_value='false',
            description=(
                'Rendering backend: '
                'true = Mesa D3D12 GPU (WSL2 + NVIDIA GPU); '
                'false = LLVMpipe software renderer (default, works on any machine)'
            ),
        ),

        # Gazebo Sim (Fortress) + robot + controllers
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_panda, 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': world_path,
                'use_sim_time': 'true',
                'use_gpu': use_gpu,
                'headless': headless,
                'rviz': rviz,
            }.items(),
        ),

        # MoveIt2 move_group + planning pipeline
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_icgnet_main, 'launch', 'move_group.launch.py')
            ),
        ),

        # ros_gz_bridge: /clock + camera topics + gz-sim entity services.
        # Camera sensor thread needs ~10s to init; bridge auto-reconnects until topic appears.
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_ros_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                '/camera/rgbd_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
                '/camera/rgbd_camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
                '/camera/rgbd_camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
                '/camera/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
                # gz-sim entity services for spawn_object + grasp_executor reset_scene
                '/world/icgnet_world/set_pose@ros_gz_interfaces/srv/SetEntityPose',
                # Entity removal — used by the Phase 1 evaluator to clear the scene between runs
                '/world/icgnet_world/remove@ros_gz_interfaces/srv/DeleteEntity',
                # Model poses: child_frame_id = model name; used by grasp_executor for
                # physical success check (_object_in_bin). Remapped to /model_poses for brevity.
                '/world/icgnet_world/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            ],
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[
                ('/world/icgnet_world/dynamic_pose/info', '/model_poses'),
            ],
        ),

        # TF: world → camera_link.
        # MUST match the camera_model pose in icgnet_world.sdf exactly.
        # Pose: [x=0.97, y=0, z=0.616], RPY=[roll=0, pitch=1.0, yaw=π]
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.97', '0', '0.616', '3.14159', '1.0', '0', 'world', 'camera_link'],
            parameters=[{'use_sim_time': True}],
        ),

        # TF: camera_link → camera_link_optical (ROS optical frame convention)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'camera_link', 'camera_link_optical'],
            parameters=[{'use_sim_time': True}],
        ),

        # TF: gz-sim publishes sensor data with frame_id = model/link/sensor path.
        # The gz-sim sensor frame matches the link's body frame (camera_link), NOT the optical frame.
        # Identity transform: camera_model/camera_link/rgbd_camera IS camera_link.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0',
                       'camera_link', 'camera_model/camera_link/rgbd_camera'],
            parameters=[{'use_sim_time': True}],
        ),

        # RViz digital twin: publishes live Gazebo mesh poses as MarkerArray on /icgnet/scene_meshes.
        # Always-on — covers GPU inference, replay, and multi-object modes identically.
        Node(
            package='icgnet_main',
            executable='scene_visualizer',
            name='scene_visualizer',
            output='screen',
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(rviz),
        ),

        # Object spawn: add objects manually after launch:
        #   ros2 run icgnet_main spawn_object --ros-args -p target_class:=can
        # Delete: ros2 service call /world/icgnet_world/remove ros_gz_interfaces/srv/DeleteEntity
        #         "{entity: {name: 'target_obj', type: 2}}"
    ])
