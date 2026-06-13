"""Scene manager node: spawns multi-object scenes and provides reset service.

Spawns T objects of the target class (entity names target_obj_0..T-1) and D
distractors from OTHER classes (entity names distractor_0..D-1).  Publishes a
latched SceneManifest that the grasp_executor uses to drive multi-object sweep.
Serves /icgnet/reset_scene (Trigger) which teleports every object back to its
original spawn pose — used by the executor on full failure recovery.
"""
import math
import os
import random
import time
from threading import Lock, Thread

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from std_srvs.srv import Trigger

from icgnet_msgs.msg import SceneManifest, SceneObject
from icgnet_main.scene_utils import (
    find_model_sdf, get_random_pose, half_height_from_sdf, load_catalog, spawn_gz_entity,
)


class SceneManagerNode(Node):
    def __init__(self):
        super().__init__('scene_manager')

        self.declare_parameter('target_class', 'can')
        self.declare_parameter('target_count', 2)
        self.declare_parameter('distractor_count', -1)  # -1 = random choice of 2 or 3
        # x range kept inside the fixed camera's reliable view. Camera at (0.97,0,0.616)
        # looks back/down (optical axis hits ground ~x=0.575); x>0.70 is the near, grazing
        # edge of the frustum where Mask3D fails to segment objects (e.g. a can at x=0.80
        # went undetected). Stay clear of it so every spawned object is seen by ICGNet.
        self.declare_parameter('spawn_x_min', 0.45)
        self.declare_parameter('spawn_x_max', 0.70)
        self.declare_parameter('spawn_y_min', -0.30)
        self.declare_parameter('spawn_y_max', 0.30)
        self.declare_parameter('spawn_reach_max', 0.85)
        self.declare_parameter('spawn_reach_min', 0.30)
        self.declare_parameter('spawn_min_dist', 0.18)

        p = self.get_parameter
        self._target_class = p('target_class').get_parameter_value().string_value
        self._target_count = p('target_count').get_parameter_value().integer_value
        self._distractor_count = p('distractor_count').get_parameter_value().integer_value
        self._x_min = p('spawn_x_min').get_parameter_value().double_value
        self._x_max = p('spawn_x_max').get_parameter_value().double_value
        self._y_min = p('spawn_y_min').get_parameter_value().double_value
        self._y_max = p('spawn_y_max').get_parameter_value().double_value
        self._reach_max = p('spawn_reach_max').get_parameter_value().double_value
        self._reach_min = p('spawn_reach_min').get_parameter_value().double_value
        self._min_dist = p('spawn_min_dist').get_parameter_value().double_value

        pkg_share = get_package_share_directory('icgnet_main')
        self._models_dir = os.path.join(pkg_share, 'models')
        self._catalog = load_catalog(self._models_dir)

        if self._target_class not in self._catalog:
            self.get_logger().error(
                f"Unknown target_class '{self._target_class}'. "
                f"Available: {list(self._catalog.keys())}"
            )
            raise RuntimeError(f"Unknown target_class '{self._target_class}'")

        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._manifest_pub = self.create_publisher(
            SceneManifest, '/icgnet/scene_manifest', latched_qos
        )

        # Service callback and entity client in separate callback groups so that
        # _reset_scene_cb can call call_async() + busy-wait without deadlock.
        cb_reset = MutuallyExclusiveCallbackGroup()
        cb_clients = ReentrantCallbackGroup()

        self._set_entity_client = self.create_client(
            SetEntityPose, '/world/icgnet_world/set_pose', callback_group=cb_clients
        )
        self.create_service(
            Trigger, '/icgnet/reset_scene', self._reset_scene_cb, callback_group=cb_reset
        )

        # Registry: list of dicts with keys entity_name, model_name, sdf_path, x, y, z, yaw,
        # semantic_class.  Written once by spawn_scene(), read by _reset_scene_cb.
        self._registry: list[dict] = []
        self._registry_lock = Lock()

    def _get_random_pose(self, existing_poses: list) -> tuple[float | None, float | None]:
        return get_random_pose(
            self._x_min, self._x_max, self._y_min, self._y_max,
            self._reach_min, self._reach_max, self._min_dist, existing_poses,
        )

    def spawn_scene(self):
        """Spawn target objects + distractors and publish the SceneManifest."""
        existing_poses: list[tuple[float, float]] = []
        registry: list[dict] = []

        self.get_logger().info('Waiting 5 s for Gazebo to be ready...')
        time.sleep(5.0)

        # ── Target objects ────────────────────────────────────────────────────
        target_models = self._catalog[self._target_class]['models']
        target_class_id = self._catalog[self._target_class]['class_id']

        for i in range(self._target_count):
            model_name = random.choice(target_models)
            entity_name = f'target_obj_{i}'

            sdf_path = find_model_sdf(self._models_dir, model_name)
            if sdf_path is None:
                self.get_logger().error(f"SDF not found for '{model_name}' — skipping {entity_name}")
                continue

            x, y = self._get_random_pose(existing_poses)
            if x is None:
                self.get_logger().error(f"No valid spawn position for {entity_name} — skipping")
                continue

            yaw = random.uniform(0, 2 * math.pi)
            z = half_height_from_sdf(sdf_path) + 0.002

            ok = spawn_gz_entity(entity_name, sdf_path, x, y, z, yaw, logger=self.get_logger())
            if ok:
                existing_poses.append((x, y))
                registry.append({
                    'entity_name': entity_name,
                    'model_name': model_name,
                    'sdf_path': sdf_path,
                    'x': x, 'y': y, 'z': z, 'yaw': yaw,
                    'semantic_class': target_class_id,
                })
                self.get_logger().info(
                    f"[SPAWN] {entity_name} ({model_name}) at ({x:.3f}, {y:.3f}, {z:.4f})"
                )
            time.sleep(0.5)

        # ── Distractors — from classes OTHER than target ───────────────────────
        n_distractors = (
            self._distractor_count if self._distractor_count >= 0
            else random.choice([2, 3])
        )

        other_entries = [
            (model, self._catalog[cls]['class_id'])
            for cls in self._catalog
            if cls != self._target_class
            for model in self._catalog[cls]['models']
        ]
        random.shuffle(other_entries)

        for i in range(n_distractors):
            if not other_entries:
                self.get_logger().warn(
                    f"No more distractor models available — spawned {i}/{n_distractors}"
                )
                break
            model_name, cls_id = other_entries[i % len(other_entries)]
            entity_name = f'distractor_{i}'

            sdf_path = find_model_sdf(self._models_dir, model_name)
            if sdf_path is None:
                self.get_logger().error(
                    f"SDF not found for '{model_name}' — skipping distractor {i}"
                )
                continue

            x, y = self._get_random_pose(existing_poses)
            if x is None:
                self.get_logger().warn(f"No valid position for {entity_name} — skipping")
                continue

            yaw = random.uniform(0, 2 * math.pi)
            z = half_height_from_sdf(sdf_path) + 0.002

            ok = spawn_gz_entity(entity_name, sdf_path, x, y, z, yaw, logger=self.get_logger())
            if ok:
                existing_poses.append((x, y))
                registry.append({
                    'entity_name': entity_name,
                    'model_name': model_name,
                    'sdf_path': sdf_path,
                    'x': x, 'y': y, 'z': z, 'yaw': yaw,
                    'semantic_class': cls_id,
                })
                self.get_logger().info(
                    f"[SPAWN] {entity_name} ({model_name}) at ({x:.3f}, {y:.3f}, {z:.4f})"
                )
            time.sleep(0.5)

        with self._registry_lock:
            self._registry = registry

        self._publish_manifest(registry)
        n_targets = sum(1 for e in registry if e['entity_name'].startswith('target_obj'))
        n_dist = sum(1 for e in registry if e['entity_name'].startswith('distractor'))
        self.get_logger().info(
            f"Scene ready: {n_targets} target ({self._target_class}) + {n_dist} distractor(s), "
            f"total={len(registry)}"
        )

    def _publish_manifest(self, registry: list[dict]):
        manifest = SceneManifest()
        manifest.header.stamp = self.get_clock().now().to_msg()
        manifest.header.frame_id = 'world'
        for entry in registry:
            obj = SceneObject()
            obj.entity_name = entry['entity_name']
            obj.model_name = entry['model_name']
            obj.semantic_class = entry['semantic_class']
            obj.pose.position.x = entry['x']
            obj.pose.position.y = entry['y']
            obj.pose.position.z = entry['z']
            # Encode yaw as quaternion (rotation about Z)
            half_yaw = entry['yaw'] / 2.0
            obj.pose.orientation.z = math.sin(half_yaw)
            obj.pose.orientation.w = math.cos(half_yaw)
            manifest.objects.append(obj)
        self._manifest_pub.publish(manifest)

    def _wait_for_future(self, future, timeout: float = 5.0) -> bool:
        deadline = time.time() + timeout
        while not future.done():
            if time.time() > deadline:
                return False
            time.sleep(0.02)
        return True

    def _reset_scene_cb(self, req: Trigger.Request, res: Trigger.Response):
        """Teleport every spawned entity back to its original pose."""
        with self._registry_lock:
            registry = list(self._registry)

        if not registry:
            res.success = True
            res.message = "Registry empty — nothing to reset"
            return res

        if not self._set_entity_client.wait_for_service(timeout_sec=3.0):
            res.success = False
            res.message = "/world/icgnet_world/set_pose service not available"
            return res

        failed: list[str] = []
        for entry in registry:
            set_req = SetEntityPose.Request()
            set_req.entity.name = entry['entity_name']
            set_req.entity.type = Entity.MODEL
            set_req.pose.position.x = entry['x']
            set_req.pose.position.y = entry['y']
            set_req.pose.position.z = entry['z']
            half_yaw = entry['yaw'] / 2.0
            set_req.pose.orientation.z = math.sin(half_yaw)
            set_req.pose.orientation.w = math.cos(half_yaw)

            fut = self._set_entity_client.call_async(set_req)
            if not self._wait_for_future(fut, timeout=5.0):
                failed.append(entry['entity_name'])
                self.get_logger().warn(
                    f"[RESET] SetEntityPose timed out for '{entry['entity_name']}'"
                )
                continue
            if not fut.result().success:
                failed.append(entry['entity_name'])
                self.get_logger().warn(
                    f"[RESET] SetEntityPose failed for '{entry['entity_name']}'"
                )
            else:
                self.get_logger().info(
                    f"[RESET] '{entry['entity_name']}' → "
                    f"({entry['x']:.3f}, {entry['y']:.3f}, {entry['z']:.3f})"
                )

        res.success = len(failed) == 0
        res.message = (
            f"Reset complete: {len(registry) - len(failed)}/{len(registry)} objects restored"
            if not failed else
            f"Partial reset — failed: {failed}"
        )
        self.get_logger().info(f"[RESET] {res.message}")
        return res


def main(args=None):
    rclpy.init(args=args)
    node = SceneManagerNode()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    exec_thread = Thread(target=executor.spin, daemon=True)
    exec_thread.start()

    try:
        node.spawn_scene()
        exec_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
