import glob
import math
import os
import random
import re
import subprocess
import time

import yaml
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def _half_height_from_sdf(sdf_path: str) -> float:
    """Return half the vertical extent of the first collision geometry in the SDF, or 0.05."""
    try:
        with open(sdf_path) as f:
            content = f.read()
        # cylinder: half_height = length/2
        m = re.search(r'<length>([\d.eE+-]+)</length>', content)
        if m:
            return float(m.group(1)) / 2.0
        # box: half_height = z-dimension / 2
        m = re.search(r'<size>([\d.eE+-]+)\s+([\d.eE+-]+)\s+([\d.eE+-]+)</size>', content)
        if m:
            return float(m.group(3)) / 2.0
        # sphere: half_height = radius
        m = re.search(r'<radius>([\d.eE+-]+)</radius>', content)
        if m:
            return float(m.group(1))
    except Exception:
        pass
    return 0.05


def _load_catalog(models_dir: str) -> dict:
    catalog_path = os.path.join(models_dir, 'catalog.yaml')
    with open(catalog_path) as f:
        return yaml.safe_load(f)


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
        self.catalog = _load_catalog(self.models_dir)

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
        for _ in range(500):
            x = random.uniform(self._spawn_x_min, self._spawn_x_max)
            y = random.uniform(self._spawn_y_min, self._spawn_y_max)
            reach = math.sqrt(x**2 + y**2)
            if reach > self._spawn_reach_max or reach < self._spawn_reach_min:
                continue
            if all(math.sqrt((x - ex)**2 + (y - ey)**2) >= self._spawn_min_dist
                   for ex, ey in existing_poses):
                return x, y
        return None, None

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

        model_sdf = os.path.join(self.models_dir, '*', model_name, 'model.sdf')
        matches = glob.glob(model_sdf)
        if not matches:
            # fallback: search two levels deep
            model_sdf_direct = os.path.join(self.models_dir, model_name, 'model.sdf')
            matches = [model_sdf_direct] if os.path.isfile(model_sdf_direct) else []

        sdf_path = matches[0] if matches else None
        half_h = _half_height_from_sdf(sdf_path) if sdf_path else 0.05
        spawn_z = half_h + 0.002

        cmd = [
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-world', 'icgnet_world',
            '-name', entity_name,
            '-x', f'{x:.3f}', '-y', f'{y:.3f}', '-z', f'{spawn_z:.4f}',
            '-Y', f'{yaw:.3f}',
        ]

        if matches:
            cmd += ['-file', matches[0]]
        else:
            self.get_logger().warn(
                f"Local SDF for '{model_name}' not found — cannot fall back to database in gz-sim."
            )

        self.get_logger().info(f'[{entity_name}] Spawning {model_name} at ({x:.2f}, {y:.2f})...')

        try:
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            for line in proc.stdout:
                self.get_logger().info(f'[{entity_name}] {line.rstrip()}')
            proc.wait()
            if proc.returncode == 0:
                self.get_logger().info(f'[{entity_name}] Spawned successfully.')
                existing_poses.append((x, y))
                if entity_name == 'target_obj':
                    pt = Point(x=float(x), y=float(y), z=float(spawn_z))
                    self._spawn_pose_pub.publish(pt)
                    self.get_logger().info(
                        f'[{entity_name}] Spawn pose published: ({x:.3f}, {y:.3f}, {spawn_z:.4f})'
                    )
            else:
                self.get_logger().error(f'[{entity_name}] Spawn failed (exit {proc.returncode}).')
        except Exception as e:
            self.get_logger().error(f'[{entity_name}] Exception: {e}')


def main():
    rclpy.init()
    node = ObjectSpawner()
    node.spawn_all()
    # Brief spin to let DDS propagate the latched spawn_pose message before shutdown.
    rclpy.spin_once(node, timeout_sec=0.5)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
