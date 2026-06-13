#!/usr/bin/env python3
import os
import csv
import time
import math
import subprocess

import rclpy
from rclpy.node import Node
from icgnet_msgs.srv import ExecuteGrasp
from tf2_msgs.msg import TFMessage

class EvaluatorPhase1(Node):
    def __init__(self):
        super().__init__('evaluator_phase1')
        self.client = self.create_client(ExecuteGrasp, '/icgnet/execute_grasp')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /icgnet/execute_grasp service...')

        self.entity_poses = {}
        self.create_subscription(TFMessage, '/model_poses', self._tf_cb, 10)

    def _tf_cb(self, msg):
        for t in msg.transforms:
            self.entity_poses[t.child_frame_id] = t.transform.translation

    def remove_all_entities(self):
        names = ['target_obj'] + [f'distractor_{i}' for i in range(4)]
        self.get_logger().info('Removing old entities from Gazebo...')
        for name in names:
            # gz-sim removal is bridged in world.launch.py as DeleteEntity (type 2 = MODEL).
            subprocess.run([
                'ros2', 'service', 'call', '/world/icgnet_world/remove',
                'ros_gz_interfaces/srv/DeleteEntity',
                f'{{entity: {{name: "{name}", type: 2}}}}'
            ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(1.0)

    def spawn_scene(self, target_class):
        self.get_logger().info(f'Spawning new scene with target {target_class}...')
        subprocess.run([
            'ros2', 'run', 'icgnet_main', 'spawn_object',
            '--ros-args', '-p', f'target_class:={target_class}',
            '-p', 'num_objects:=5'
        ])
        time.sleep(2.0)

    def execute_grasp(self, target_class):
        req = ExecuteGrasp.Request()
        req.target = target_class
        req.max_attempts = 5
        req.skip_place = False

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main():
    rclpy.init()
    node = EvaluatorPhase1()

    os.makedirs('log', exist_ok=True)
    csv_file = 'log/phase1_results.csv'

    classes = ['can', 'box', 'ball']
    runs_per_class = 20

    with open(csv_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Run_ID', 'Target_Class', 'Success', 'Attempts',
                         'Planning_Time', 'Execution_Time', 'Collision_Detected',
                         'Target_Not_Found', 'Disturbance_Max_M'])

        run_id = 0
        for cls in classes:
            for i in range(runs_per_class):
                run_id += 1
                node.get_logger().info(f"========== RUN {run_id}/60: Target={cls} ==========")

                node.remove_all_entities()
                node.spawn_scene(cls)

                # Wait to collect initial poses
                t_end = time.time() + 2.0
                while time.time() < t_end:
                    rclpy.spin_once(node, timeout_sec=0.1)

                initial_poses = {}
                for j in range(4):
                    dname = f'distractor_{j}'
                    if dname in node.entity_poses:
                        p = node.entity_poses[dname]
                        initial_poses[dname] = (p.x, p.y, p.z)

                res = node.execute_grasp(cls)

                # Check disturbance
                t_end = time.time() + 2.0
                while time.time() < t_end:
                    rclpy.spin_once(node, timeout_sec=0.1)

                max_dist = 0.0
                for dname, initial_pos in initial_poses.items():
                    if dname in node.entity_poses:
                        p = node.entity_poses[dname]
                        dist = math.sqrt((p.x - initial_pos[0])**2 +
                                         (p.y - initial_pos[1])**2 +
                                         (p.z - initial_pos[2])**2)
                        if dist > max_dist:
                            max_dist = dist

                if res:
                    writer.writerow([run_id, cls, 1 if res.success else 0,
                                     res.grasps_attempted, round(res.planning_time, 2),
                                     round(res.execution_time, 2), 1 if res.collision_detected else 0,
                                     1 if res.target_not_found else 0, round(max_dist, 3)])
                    node.get_logger().info(
                        f"Run {run_id} Result: success={res.success}, attempts={res.grasps_attempted}, "
                        f"collision={res.collision_detected}, not_found={res.target_not_found}, "
                        f"max_disturbance={max_dist:.3f}m"
                    )
                else:
                    writer.writerow([run_id, cls, 0, 0, 0.0, 0.0, 0, 0, 0.0])
                f.flush()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
