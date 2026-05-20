import time
from threading import Lock, Thread

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from std_srvs.srv import Trigger

from pymoveit2 import MoveIt2, MoveIt2Gripper
from pymoveit2.robots import panda as robot

from icgnet_msgs.msg import GraspArray
from icgnet_msgs.srv import ExecuteGrasp

SEMANTIC_CLASSES = {
    'mug': 0, 'box': 1, 'can': 2, 'bottle': 3,
    'cylindric': 4, 'ball': 5, 'other': 6,
}


class GraspExecutorNode(Node):
    def __init__(self):
        super().__init__('grasp_executor_node')

        self.declare_parameter('rich_grasp_topic', '/icgnet/grasps_rich')
        self.declare_parameter('compute_grasp_service', '/icgnet/compute_grasps')
        self.declare_parameter('default_min_score', 0.4)
        self.declare_parameter('default_max_attempts', 5)
        self.declare_parameter('approach_offset', 0.10)
        self.declare_parameter('pre_grasp_z_offset', 0.10)
        self.declare_parameter('grasp_z_correction', 0.0)
        self.declare_parameter('lift_height', 0.25)
        self.declare_parameter('workspace_x_min', 0.20)
        self.declare_parameter('workspace_x_max', 0.80)
        self.declare_parameter('workspace_y_min', -0.40)
        self.declare_parameter('workspace_y_max', 0.40)
        self.declare_parameter('workspace_z_min', 0.01)
        self.declare_parameter('workspace_z_max', 0.60)

        self._approach_offset = self.get_parameter('approach_offset').get_parameter_value().double_value
        self._pre_grasp_z_offset = self.get_parameter('pre_grasp_z_offset').get_parameter_value().double_value
        self._lift_height = self.get_parameter('lift_height').get_parameter_value().double_value
        self._default_min_score = self.get_parameter('default_min_score').get_parameter_value().double_value
        self._default_max_attempts = self.get_parameter('default_max_attempts').get_parameter_value().integer_value
        self._grasp_z_correction = self.get_parameter('grasp_z_correction').get_parameter_value().double_value
        self._ws = {
            'x': (self.get_parameter('workspace_x_min').get_parameter_value().double_value,
                  self.get_parameter('workspace_x_max').get_parameter_value().double_value),
            'y': (self.get_parameter('workspace_y_min').get_parameter_value().double_value,
                  self.get_parameter('workspace_y_max').get_parameter_value().double_value),
            'z': (self.get_parameter('workspace_z_min').get_parameter_value().double_value,
                  self.get_parameter('workspace_z_max').get_parameter_value().double_value),
        }

        cb_arm = ReentrantCallbackGroup()
        cb_gripper = ReentrantCallbackGroup()
        cb_svc = ReentrantCallbackGroup()
        cb_sub = ReentrantCallbackGroup()

        self._arm = MoveIt2(
            node=self,
            joint_names=robot.joint_names(),
            base_link_name=robot.base_link_name(),
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=cb_arm,
        )
        self._arm.max_velocity = 0.2
        self._arm.max_acceleration = 0.2
        self._arm.orientation_tolerance = 0.05

        self._gripper = MoveIt2Gripper(
            node=self,
            gripper_joint_names=robot.gripper_joint_names(),
            open_gripper_joint_positions=robot.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=robot.CLOSED_GRIPPER_JOINT_POSITIONS,
            gripper_group_name=robot.MOVE_GROUP_GRIPPER,
            callback_group=cb_gripper,
        )

        compute_svc = self.get_parameter('compute_grasp_service').get_parameter_value().string_value
        self._compute_client = self.create_client(Trigger, compute_svc, callback_group=cb_svc)

        rich_topic = self.get_parameter('rich_grasp_topic').get_parameter_value().string_value
        self._latest_grasps: GraspArray | None = None
        self._grasps_lock = Lock()
        self.create_subscription(GraspArray, rich_topic, self._grasps_cb, 10, callback_group=cb_sub)

        self.create_service(ExecuteGrasp, '/icgnet/execute_grasp', self._execute_grasp_cb, callback_group=cb_svc)

        self.get_logger().info('GraspExecutorNode ready.')

    def _grasps_cb(self, msg: GraspArray):
        with self._grasps_lock:
            self._latest_grasps = msg

    def _execute_grasp_cb(self, req: ExecuteGrasp.Request, res: ExecuteGrasp.Response):
        target = req.target.strip() if req.target else 'any'
        min_score = req.min_score if req.min_score > 0.0 else self._default_min_score
        max_attempts = req.max_attempts if req.max_attempts > 0 else self._default_max_attempts

        self.get_logger().info(
            f"ExecuteGrasp: target='{target}' min_score={min_score:.2f} max_attempts={max_attempts}"
        )

        # Clear stale grasps so we know the next GraspArray is fresh
        with self._grasps_lock:
            self._latest_grasps = None

        # Trigger ICGNet inference
        if not self._compute_client.wait_for_service(timeout_sec=5.0):
            res.success = False
            res.message = f"Service '{self._compute_client.srv_name}' not available"
            return res

        future = self._compute_client.call_async(Trigger.Request())
        while not future.done():
            time.sleep(0.05)
        if not future.result().success:
            res.success = False
            res.message = f"ICGNet inference failed: {future.result().message}"
            return res

        # Wait for GraspArray published after inference
        deadline = time.time() + 5.0
        grasps = None
        while time.time() < deadline:
            with self._grasps_lock:
                grasps = self._latest_grasps
            if grasps is not None:
                break
            time.sleep(0.1)

        if grasps is None or len(grasps.grasps) == 0:
            res.success = False
            res.message = "No grasps received after inference"
            return res

        candidates = self._filter_grasps(grasps.grasps, target, min_score)
        if not candidates:
            res.success = False
            res.message = f"No grasps after filtering: target='{target}' min_score={min_score:.2f}"
            return res

        candidates = candidates[:max_attempts]
        self.get_logger().info(f"{len(candidates)} candidates, attempting up to {max_attempts}")

        self._gripper.open()
        self._gripper.wait_until_executed()

        for i, g in enumerate(candidates):
            self.get_logger().info(
                f"Attempt {i+1}/{len(candidates)}: score={g.score:.2f} "
                f"inst={g.instance_id} cls={g.semantic_class}"
            )
            if self._execute_single_grasp(g):
                res.success = True
                res.grasps_attempted = i + 1
                res.message = f"Grasp succeeded on attempt {i+1}"
                return res
            # Re-open for next attempt
            self._gripper.open()
            self._gripper.wait_until_executed()

        res.success = False
        res.grasps_attempted = len(candidates)
        res.message = f"All {len(candidates)} grasp attempts failed"
        return res

    def _filter_grasps(self, grasps, target: str, min_score: float) -> list:
        filtered = []
        for g in grasps:
            p = g.pose.position
            if g.score < min_score:
                continue
            if g.width > 0.08:
                continue
            if not (self._ws['x'][0] <= p.x <= self._ws['x'][1]):
                continue
            if not (self._ws['y'][0] <= p.y <= self._ws['y'][1]):
                continue
            if not (self._ws['z'][0] <= p.z <= self._ws['z'][1]):
                continue
            if not self._matches_target(g, target):
                continue
            filtered.append(g)
        filtered.sort(key=lambda g: g.score, reverse=True)
        return filtered

    def _matches_target(self, g, target: str) -> bool:
        if target == 'any':
            return True
        if target.startswith('instance_'):
            try:
                return g.instance_id == int(target.split('_', 1)[1])
            except ValueError:
                self.get_logger().warn(f"Invalid instance target '{target}'")
                return False
        if target in SEMANTIC_CLASSES:
            return g.semantic_class == SEMANTIC_CLASSES[target]
        self.get_logger().warn(f"Unknown target '{target}' — treating as 'any'")
        return True

    def _execute_single_grasp(self, g) -> bool:
        pos = np.array([g.pose.position.x, g.pose.position.y, g.pose.position.z + self._grasp_z_correction])
        q = g.pose.orientation
        quat_xyzw = [q.x, q.y, q.z, q.w]
        approach = Rotation.from_quat(quat_xyzw).as_matrix()[:, 2]  # z-col = approach axis

        # Pre-grasp: pull back along approach axis AND lift above grasp z.
        # Lifting avoids near-floor arm configurations where the effort JTC
        # can't maintain torque balance (joint4 near its -3.07 rad limit).
        z_bias = np.array([0.0, 0.0, self._pre_grasp_z_offset])
        pre_pos = (pos - self._approach_offset * approach + z_bias).tolist()

        self.get_logger().info(
            f"[DIAG] grasp_pos=[{pos[0]:.4f},{pos[1]:.4f},{pos[2]:.4f}] "
            f"pre_pos=[{pre_pos[0]:.4f},{pre_pos[1]:.4f},{pre_pos[2]:.4f}] "
            f"quat=[{quat_xyzw[0]:.4f},{quat_xyzw[1]:.4f},{quat_xyzw[2]:.4f},{quat_xyzw[3]:.4f}] "
            f"approach=[{approach[0]:.4f},{approach[1]:.4f},{approach[2]:.4f}] "
            f"width={g.width:.4f}"
        )

        t0 = time.time()
        self._arm.move_to_pose(position=pre_pos, quat_xyzw=quat_xyzw)
        ok = self._arm.wait_until_executed()
        dt = time.time() - t0
        if not ok:
            self.get_logger().warn(
                f"[DIAG] PRE-GRASP FAILED in {dt:.2f}s "
                f"target=[{pre_pos[0]:.4f},{pre_pos[1]:.4f},{pre_pos[2]:.4f}]"
            )
            return False
        self.get_logger().info(f"[DIAG] pre-grasp OK in {dt:.2f}s")

        t0 = time.time()
        self._arm.move_to_pose(position=pos.tolist(), quat_xyzw=quat_xyzw)
        ok = self._arm.wait_until_executed()
        dt = time.time() - t0
        if not ok:
            self.get_logger().warn(
                f"[DIAG] APPROACH FAILED in {dt:.2f}s "
                f"target=[{pos[0]:.4f},{pos[1]:.4f},{pos[2]:.4f}]"
            )
            return False
        self.get_logger().info(f"[DIAG] approach OK in {dt:.2f}s")

        self._gripper.close()
        self._gripper.wait_until_executed()

        lift_pos = (pos + np.array([0.0, 0.0, self._lift_height])).tolist()
        t0 = time.time()
        self._arm.move_to_pose(position=lift_pos, quat_xyzw=quat_xyzw)
        self._arm.wait_until_executed()
        self.get_logger().info(f"[DIAG] lift done in {time.time()-t0:.2f}s")

        return True


def main(args=None):
    rclpy.init(args=args)
    node = GraspExecutorNode()
    executor = MultiThreadedExecutor(4)
    executor.add_node(node)
    thread = Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
