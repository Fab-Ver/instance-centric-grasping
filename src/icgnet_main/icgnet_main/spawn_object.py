import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import os
import random
import math
import time
import subprocess
import sys

class ObjectSpawner(Node):
    def __init__(self):
        super().__init__('object_spawner')
        self.declare_parameter('target_type', 'coke_can')
        self.declare_parameter('num_objects', 1)
        self.declare_parameter('mode', 'offline')

        self.target_type = str(self.get_parameter('target_type').value)
        try:
            self.num_objects = int(self.get_parameter('num_objects').value)
        except:
            self.num_objects = 1
        self.mode = str(self.get_parameter('mode').value)
        
        # Valid classes
        self.categories = ['coke_can', 'beer', 'bowl', 'cricket_ball', 'wood_cube_10cm']
        
        self.get_logger().info('==========================================')
        self.get_logger().info(f'SPAWNER MODE: {"MULTI" if self.num_objects > 1 else "SINGLE"}')
        self.get_logger().info(f'Target: {self.target_type}')
        self.get_logger().info(f'Total Objects: {self.num_objects}')
        self.get_logger().info('==========================================')

    def get_random_pose(self, existing_poses, min_dist=0.18):
        for _ in range(500):
            x = random.uniform(0.40, 0.80)
            y = random.uniform(-0.30, 0.30)
            dist_base = math.sqrt(x**2 + y**2)
            if dist_base > 0.85 or dist_base < 0.30:
                continue
            
            too_close = False
            for ex_x, ex_y in existing_poses:
                if math.sqrt((x - ex_x)**2 + (y - ex_y)**2) < min_dist:
                    too_close = True
                    break
            if not too_close:
                return x, y
        return None, None

    def spawn_all(self):
        existing_poses = []
        self.get_logger().info('Waiting 10s for Gazebo...')
        time.sleep(10.0)
        
        # ALWAYS spawn the target first
        self.spawn_one(self.target_type, 'target_obj', existing_poses, is_target=True)
        
        # Spawn distractors ONLY IF num_objects > 1
        if self.num_objects > 1:
            self.get_logger().info(f'Spawning {self.num_objects - 1} distractors...')
            for i in range(self.num_objects - 1):
                self.get_logger().info('Waiting 5s for next distractor...')
                time.sleep(5.0)
                obj_type = random.choice(self.categories)
                # Avoid target as distractor if possible
                if obj_type == self.target_type:
                    others = [c for c in self.categories if c != self.target_type]
                    obj_type = random.choice(others) if others else self.categories[0]
                self.spawn_one(obj_type, f'distractor_{i}', existing_poses)
        else:
            self.get_logger().info('Single object mode: skipping distractors.')

    def spawn_one(self, model_name, entity_name, existing_poses, is_target=False):
        x, y = self.get_random_pose(existing_poses)
        if x is None:
            self.get_logger().error(f'No space for {entity_name}')
            return

        yaw = random.uniform(0, 2 * math.pi)
        
        cmd = [
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-entity', entity_name,
            '-x', f'{x:.3f}', '-y', f'{y:.3f}', '-z', '0.42',
            '-Y', f'{yaw:.3f}'
        ]

        if self.mode == 'offline' and is_target and model_name == 'coke_can':
            pkg_path = get_package_share_directory('icgnet_main')
            model_path = os.path.join(pkg_path, 'models', 'coke_can', 'model.sdf')
            cmd += ['-file', model_path]
        else:
            cmd += ['-database', model_name]

        self.get_logger().info(f'[{entity_name}] Spawning {model_name}...')
        
        try:
            process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            while True:
                line = process.stdout.readline()
                if not line and process.poll() is not None:
                    break
                if line:
                    self.get_logger().info(f'[{entity_name}] {line.strip()}')
            
            if process.returncode == 0:
                self.get_logger().info(f'[{entity_name}] SUCCESS.')
                existing_poses.append((x, y))
            else:
                self.get_logger().error(f'[{entity_name}] FAILED.')
        except Exception as e:
            self.get_logger().error(f'[{entity_name}] EXCEPTION: {str(e)}')

def main():
    rclpy.init()
    node = ObjectSpawner()
    node.spawn_all()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
