import math
import os
import random
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from icgnet_main.scene_utils import (
    find_model_sdf, get_random_pose, half_height_from_sdf, load_catalog, spawn_gz_entity,
)


class ObjectSpawner(Node):
    def __init__(self):
        super().__init__('object_spawner')
        self.declare_parameter('target_type', '')
        self.declare_parameter('target_class', 'can')
        self.declare_parameter('num_objects', 1)
        self.declare_parameter('spawn_x_min', 0.40)
        self.declare_parameter('spawn_x_max', 0.80)
        self.declare_parameter('spawn_y_min', -0.30)
        self.declare_parameter('spawn_y_max', 0.30)
        self.declare_parameter('spawn_reach_max', 0.85)
        self.declare_parameter('spawn_reach_min', 0.30)
        self.declare_parameter('spawn_min_dist', 0.18)

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

        self._spawn_pose_pub = self.create_publisher(
            Point,
            '/icgnet/object_spawn_pose',
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )

    def _get_random_pose(self, existing_poses: list) -> tuple[float | None, float | None]:
        return get_random_pose(
            self._spawn_x_min, self._spawn_x_max,
            self._spawn_y_min, self._spawn_y_max,
            self._spawn_reach_min, self._spawn_reach_max,
            self._spawn_min_dist, existing_poses,
        )

    def spawn_all(self):
        existing_poses = []
        self.get_logger().info('Waiting 5s for Gazebo...')
        time.sleep(5.0)

        self._spawn_one(self._resolved_target, 'target_obj', existing_poses, fixed_pos=None)

        if self.num_objects > 1:
            self.get_logger().info(f'Spawning {self.num_objects - 1} distractors...')
            for i in range(self.num_objects - 1):
                time.sleep(0.5)
                others = [m for m in self._all_models if m != self._resolved_target]
                distractor = random.choice(others) if others else self._all_models[0]
                self._spawn_one(distractor, f'distractor_{i}', existing_poses)
        else:
            self.get_logger().info('Single object mode: no distractors.')

    def _spawn_one(self, model_name: str, entity_name: str, existing_poses: list, fixed_pos=None):
        if fixed_pos is not None:
            x, y = fixed_pos
        else:
            x, y = self._get_random_pose(existing_poses)
            if x is None:
                self.get_logger().error(f'No valid spawn position for {entity_name}. Skipping.')
                return

        yaw = random.uniform(0, 2 * math.pi)

        sdf_path = find_model_sdf(self.models_dir, model_name)
        if sdf_path is None:
            self.get_logger().warn(
                f"Local SDF for '{model_name}' not found — cannot fall back to database in gz-sim."
            )
            return

        half_h = half_height_from_sdf(sdf_path)
        spawn_z = half_h + 0.002

        self.get_logger().info(f'[{entity_name}] Spawning {model_name} at ({x:.2f}, {y:.2f})...')

        ok = spawn_gz_entity(entity_name, sdf_path, x, y, spawn_z, yaw, logger=self.get_logger())
        if ok:
            self.get_logger().info(f'[{entity_name}] Spawned successfully.')
            existing_poses.append((x, y))
            if entity_name == 'target_obj':
                pt = Point(x=float(x), y=float(y), z=float(spawn_z))
                self._spawn_pose_pub.publish(pt)
                self.get_logger().info(
                    f'[{entity_name}] Spawn pose published: ({x:.3f}, {y:.3f}, {spawn_z:.4f})'
                )
        else:
            self.get_logger().error(f'[{entity_name}] Spawn failed.')


def main():
    rclpy.init()
    node = ObjectSpawner()
    node.spawn_all()
    # Brief spin to let DDS propagate the latched spawn_pose message before shutdown.
    rclpy.spin_once(node, timeout_sec=0.5)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
