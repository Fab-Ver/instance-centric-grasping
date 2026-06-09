import json
import os
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message

from moveit_msgs.msg import CollisionObject
from icgnet_msgs.msg import GraspArray, SceneManifest, SceneObject
from std_srvs.srv import Trigger

from icgnet_main.scene_utils import find_model_sdf


class ReplayInferenceNode(Node):
    def __init__(self):
        super().__init__('replay_inference_node')

        self.declare_parameter('inference_dir', os.path.expanduser('~/icgnet_inference_data'))
        self.declare_parameter('spawn_object', True)

        self._inference_dir = self.get_parameter('inference_dir').get_parameter_value().string_value
        self._spawn_object_flag = self.get_parameter('spawn_object').get_parameter_value().bool_value

        self._grasps: GraspArray | None = None
        self._collision_objects: list[CollisionObject] = []
        self._meta: dict = {}
        self._has_spawned = False
        self._load_data()

        cb = ReentrantCallbackGroup()
        qos_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        pkg_share = get_package_share_directory('icgnet_main')
        self._models_dir = os.path.join(pkg_share, 'models')

        self._grasps_pub = self.create_publisher(GraspArray, '/icgnet/grasps_rich', 10)
        self._co_pub = self.create_publisher(CollisionObject, '/collision_object', qos_reliable)
        self._manifest_viz_pub = self.create_publisher(
            SceneManifest, '/icgnet/scene_manifest_viz', latched_qos,
        )

        self.create_service(Trigger, '/icgnet/compute_grasps', self._trigger_cb, callback_group=cb)

        n_grasps = len(self._grasps.grasps) if self._grasps else 0
        self.get_logger().info(
            f'ReplayInferenceNode ready — {n_grasps} grasps, '
            f'{len(self._collision_objects)} collision objects from {self._inference_dir}\n'
            'Trigger: ros2 service call /icgnet/compute_grasps std_srvs/srv/Trigger'
        )

    def _load_data(self):
        meta_path = os.path.join(self._inference_dir, 'meta.json')
        if not os.path.exists(meta_path):
            self.get_logger().error(f'meta.json not found: {meta_path}')
            return

        with open(meta_path) as f:
            self._meta = json.load(f)

        grasps_path = os.path.join(self._inference_dir, 'grasps.bin')
        with open(grasps_path, 'rb') as f:
            self._grasps = deserialize_message(f.read(), GraspArray)
        self.get_logger().info(f'Loaded {len(self._grasps.grasps)} grasps from {grasps_path}')

        for idx in range(self._meta.get('n_collision_objects', 0)):
            co_path = os.path.join(self._inference_dir, f'collision_{idx}.bin')
            with open(co_path, 'rb') as f:
                co = deserialize_message(f.read(), CollisionObject)
            self._collision_objects.append(co)
            self.get_logger().info(f'  Loaded collision object: {co.id}')

    def _trigger_cb(self, _request, response):
        if self._grasps is None:
            response.success = False
            response.message = 'No inference data loaded.'
            return response

        if self._spawn_object_flag and not self._has_spawned:
            self._spawn_object()
            self._has_spawned = True

        for co in self._collision_objects:
            self._co_pub.publish(co)

        # Let collision objects reach move_group before publishing grasps.
        time.sleep(0.2)

        self._grasps_pub.publish(self._grasps)

        response.success = True
        response.message = (
            f'Replayed {len(self._grasps.grasps)} grasps + '
            f'{len(self._collision_objects)} collision objects.'
        )
        self.get_logger().info(response.message)
        return response

    def _spawn_object(self):
        """Spawn the saved object into gz-sim via ros_gz_sim create."""
        sdf_path = self._meta.get('object_sdf_path', '')
        if not sdf_path or not os.path.exists(sdf_path):
            self.get_logger().warn(f'SDF not found: {sdf_path!r} — skipping object spawn.')
            return

        pose = self._meta.get('object_pose', {})
        name = self._meta.get('object_name', 'target_obj')
        x = float(pose.get('x', 0.65))
        y = float(pose.get('y', 0.0))
        z = float(pose.get('z', 0.05))

        cmd = [
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-world', 'icgnet_world',
            '-name', name,
            '-file', sdf_path,
            '-x', f'{x:.3f}', '-y', f'{y:.3f}', '-z', f'{z:.4f}',
        ]

        self.get_logger().info(f'Spawning {name} at ({x:.2f}, {y:.2f}, {z:.4f})...')
        try:
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            for line in proc.stdout:
                self.get_logger().info(f'[spawn] {line.rstrip()}')
            proc.wait()
            if proc.returncode == 0:
                self.get_logger().info(f'Spawned {name}.')
                self._publish_manifest_viz(name, sdf_path, x, y, z)
            else:
                self.get_logger().warn(f'Spawn failed (exit {proc.returncode}).')
        except Exception as e:
            self.get_logger().error(f'Spawn exception: {e}')

    def _publish_manifest_viz(self, entity_name: str, sdf_path: str, x: float, y: float, z: float):
        """Publish a 1-entry latched manifest so scene_visualizer can render this object."""
        model_name = os.path.basename(os.path.dirname(sdf_path))
        # Confirm the model is resolvable from the installed share (needed by viz node).
        if find_model_sdf(self._models_dir, model_name) is None:
            self.get_logger().warn(
                f'[VIZ_MANIFEST] Model {model_name!r} not found in share — mesh will not render.'
            )
        manifest = SceneManifest()
        manifest.header.frame_id = 'world'
        obj = SceneObject()
        obj.entity_name = entity_name
        obj.model_name = model_name
        obj.semantic_class = 0
        obj.pose.position.x = x
        obj.pose.position.y = y
        obj.pose.position.z = z
        obj.pose.orientation.w = 1.0
        manifest.objects.append(obj)
        self._manifest_viz_pub.publish(manifest)
        self.get_logger().info(
            f'[VIZ_MANIFEST] Published {entity_name}({model_name}) to /icgnet/scene_manifest_viz.'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ReplayInferenceNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
