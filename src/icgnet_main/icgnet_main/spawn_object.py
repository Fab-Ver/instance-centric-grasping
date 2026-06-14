import math
import os
import random
import sys
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from icgnet_msgs.msg import SceneManifest, SceneObject
from icgnet_main.scene_utils import (
    find_model_sdf, get_random_pose, half_height_from_sdf, load_catalog, spawn_gz_entity,
)


class ObjectSpawner(Node):
    def __init__(self):
        super().__init__('object_spawner')
        self.declare_parameter('target_type', '')
        self.declare_parameter('target_class', 'can')
        self.declare_parameter('num_objects', 1)
        # Spawn region restricted to the arm's dexterous workspace AND the camera's
        # reliable FOV (matches scene_manager). x>0.70 is the grazing frustum edge where
        # Mask3D fails to segment (a can at x=0.80 went undetected); reach>0.75 puts the
        # object near the Panda's kinematic limit where even a vertical grasp is unplannable
        # (inflates PREGRASP_PLAN_FAIL with failures not attributable to ICGNet). Keeping
        # spawns inside this box isolates ICGNet grasp quality from camera/kinematic confounds.
        self.declare_parameter('spawn_x_min', 0.45)
        self.declare_parameter('spawn_x_max', 0.70)
        self.declare_parameter('spawn_y_min', -0.30)
        self.declare_parameter('spawn_y_max', 0.30)
        self.declare_parameter('spawn_reach_max', 0.75)
        self.declare_parameter('spawn_reach_min', 0.30)
        self.declare_parameter('spawn_min_dist', 0.18)
        # Seconds to wait for the gz-sim server to be ready before spawning. Needed only on a
        # cold start (spawn_object launched right after world.launch.py). In the evaluation
        # loop the server is already up and presence is verified via /model_poses, so the
        # harness sets this to 0 to avoid wasting ~5s per run.
        self.declare_parameter('gz_server_wait', 5.0)

        self.target_type = self.get_parameter('target_type').get_parameter_value().string_value.strip()
        self.target_class = self.get_parameter('target_class').get_parameter_value().string_value.strip()
        self.num_objects = self.get_parameter('num_objects').get_parameter_value().integer_value
        self._spawn_x_min = self.get_parameter('spawn_x_min').get_parameter_value().double_value
        self._spawn_x_max = self.get_parameter('spawn_x_max').get_parameter_value().double_value
        self._spawn_y_min = self.get_parameter('spawn_y_min').get_parameter_value().double_value
        self._spawn_y_max = self.get_parameter('spawn_y_max').get_parameter_value().double_value
        self._spawn_reach_max = self.get_parameter('spawn_reach_max').get_parameter_value().double_value
        self._spawn_reach_min = self.get_parameter('spawn_reach_min').get_parameter_value().double_value
        self._spawn_min_dist = self.get_parameter('spawn_min_dist').get_parameter_value().double_value
        self._gz_server_wait = self.get_parameter('gz_server_wait').get_parameter_value().double_value

        pkg_share = get_package_share_directory('icgnet_main')
        self.models_dir = os.path.join(pkg_share, 'models')
        self.catalog = load_catalog(self.models_dir)

        # Resolve target model name
        if self.target_type:
            # Legacy: exact model name supplied
            self._resolved_target = self.target_type
        elif self.target_class in self.catalog:
            self._resolved_target = random.choice(self.catalog[self.target_class]['models'])
        else:
            self.get_logger().warn(
                f"Unknown target_class '{self.target_class}', defaulting to 'coke_can'."
            )
            self._resolved_target = 'coke_can'

        # Flat list of all available model names for distractors
        self._all_models = [m for cls in self.catalog.values() for m in cls['models']]

        self.get_logger().info('==========================================')
        self.get_logger().info(f'SPAWNER MODE: {"MULTI" if self.num_objects > 1 else "SINGLE"}')
        self.get_logger().info(f'Target model: {self._resolved_target}')
        self.get_logger().info(f'Total objects: {self.num_objects}')
        self.get_logger().info('==========================================')

        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # Viz-only manifest: lets scene_visualizer map entity→model in single/multi-spawn modes.
        # Published on a separate topic so grasp_executor does not treat it as a multi-object sweep.
        self._manifest_viz_pub = self.create_publisher(SceneManifest, '/icgnet/scene_manifest_viz', latched_qos)
        self._spawned_entries: list[dict] = []

        # Build reverse mapping model_name → semantic class id for manifest population.
        self._model_to_class: dict[str, int] = {
            m: cls_data['class_id']
            for cls_data in self.catalog.values()
            for m in cls_data['models']
        }

    def _get_random_pose(self, existing_poses: list) -> tuple[float | None, float | None]:
        return get_random_pose(
            self._spawn_x_min, self._spawn_x_max,
            self._spawn_y_min, self._spawn_y_max,
            self._spawn_reach_min, self._spawn_reach_max,
            self._spawn_min_dist, existing_poses,
        )

    def spawn_all(self) -> bool:
        """Spawn the target (+ distractors). Returns True iff the target object spawned."""
        existing_poses = []
        if self._gz_server_wait > 0.0:
            self.get_logger().info(f'Waiting {self._gz_server_wait:.1f}s for gz-sim server...')
            time.sleep(self._gz_server_wait)

        target_ok = self._spawn_one(self._resolved_target, 'target_obj', existing_poses, fixed_pos=None)

        if self.num_objects > 1:
            self.get_logger().info(f'Spawning {self.num_objects - 1} distractors...')
            for i in range(self.num_objects - 1):
                time.sleep(0.5)
                others = [m for m in self._all_models if m != self._resolved_target]
                distractor = random.choice(others) if others else self._all_models[0]
                self._spawn_one(distractor, f'distractor_{i}', existing_poses)
        else:
            self.get_logger().info('Single object mode: no distractors.')

        return target_ok

    def _spawn_one(self, model_name: str, entity_name: str, existing_poses: list, fixed_pos=None) -> bool:
        if fixed_pos is not None:
            x, y = fixed_pos
        else:
            x, y = self._get_random_pose(existing_poses)
            if x is None:
                self.get_logger().error(f'No valid spawn position for {entity_name}. Skipping.')
                return False

        yaw = random.uniform(0, 2 * math.pi)

        sdf_path = find_model_sdf(self.models_dir, model_name)
        if sdf_path is None:
            self.get_logger().warn(
                f"Local SDF for '{model_name}' not found — cannot fall back to database in gz-sim."
            )
            return False

        half_h = half_height_from_sdf(sdf_path)
        spawn_z = half_h + 0.002

        self.get_logger().info(f'[{entity_name}] Spawning {model_name} at ({x:.2f}, {y:.2f})...')

        ok = spawn_gz_entity(entity_name, sdf_path, x, y, spawn_z, yaw, logger=self.get_logger())
        if ok:
            self.get_logger().info(f'[{entity_name}] Spawned successfully.')
            existing_poses.append((x, y))
            self._spawned_entries.append({
                'entity_name': entity_name,
                'model_name': model_name,
                'semantic_class': self._model_to_class.get(model_name, 0),
                'x': x, 'y': y, 'z': spawn_z, 'yaw': yaw,
            })
        else:
            self.get_logger().error(f'[{entity_name}] Spawn failed.')
        return ok


    def _publish_manifest_viz(self):
        """Publish a latched SceneManifest for scene_visualizer covering all spawned entities."""
        if not self._spawned_entries:
            return
        manifest = SceneManifest()
        manifest.header.frame_id = 'world'
        for entry in self._spawned_entries:
            obj = SceneObject()
            obj.entity_name = entry['entity_name']
            obj.model_name = entry['model_name']
            obj.semantic_class = entry['semantic_class']
            obj.pose.position.x = float(entry['x'])
            obj.pose.position.y = float(entry['y'])
            obj.pose.position.z = float(entry['z'])
            half_yaw = float(entry['yaw']) / 2.0
            obj.pose.orientation.z = math.sin(half_yaw)
            obj.pose.orientation.w = math.cos(half_yaw)
            manifest.objects.append(obj)
        self._manifest_viz_pub.publish(manifest)
        self.get_logger().info(
            f'[VIZ_MANIFEST] Published {len(manifest.objects)} entity/-ies to /icgnet/scene_manifest_viz.'
        )


def main():
    rclpy.init()
    node = ObjectSpawner()
    target_ok = node.spawn_all()
    node._publish_manifest_viz()
    # Brief spin to let DDS propagate the latched messages before shutdown.
    rclpy.spin_once(node, timeout_sec=0.5)
    rclpy.shutdown()
    # Non-zero exit on target spawn failure (e.g. name collision with a stale entity)
    # so the evaluation harness can detect it and retry instead of grasping a stale scene.
    sys.exit(0 if target_ok else 1)


if __name__ == '__main__':
    main()
