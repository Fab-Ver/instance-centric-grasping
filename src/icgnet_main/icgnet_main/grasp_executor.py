import time
from threading import Lock, Thread

import numpy as np
import rclpy
import rclpy.duration
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Point
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from pymoveit2 import MoveIt2, MoveIt2Gripper
from pymoveit2.robots import panda as robot

from icgnet_msgs.msg import GraspArray
from icgnet_msgs.srv import ExecuteGrasp

# Compact home configuration matching the URDF initial_value parameters
HOME_JOINT_POSITIONS = [0.0, -1.0, 0.0, -2.5, 0.0, 2.0, 0.785]

SEMANTIC_CLASSES = {
    'mug': 0, 'box': 1, 'can': 2, 'bottle': 3,
    'cylindric': 4, 'ball': 5, 'other': 6,
}


def _gripper_points_world(c: np.ndarray, R: np.ndarray, w: float):
    """Gripper wireframe keypoints in world frame. Matches grasp_service_node geometry."""
    half_w = w / 2.0
    lf_base = np.array([0.0,  half_w, -0.045])
    rf_base = np.array([0.0, -half_w, -0.045])
    lf_tip  = np.array([0.0,  half_w,  0.005])
    rf_tip  = np.array([0.0, -half_w,  0.005])
    cb      = np.array([0.0,  0.0,    -0.045])
    handle  = np.array([0.0,  0.0,    -0.115])

    def wp(lp):
        return c + R @ lp

    return [
        (wp(lf_base), wp(lf_tip)),
        (wp(rf_base), wp(rf_tip)),
        (wp(rf_base), wp(lf_base)),
        (wp(cb),      wp(handle)),
    ]


class GraspExecutorNode(Node):
    def __init__(self):
        super().__init__('grasp_executor_node')

        self.declare_parameter('rich_grasp_topic', '/icgnet/grasps_rich')
        self.declare_parameter('compute_grasp_service', '/icgnet/compute_grasps')
        self.declare_parameter('default_min_score', 0.4)
        self.declare_parameter('default_max_attempts', 5)
        self.declare_parameter('approach_offset', 0.10)
        self.declare_parameter('lift_height', 0.25)
        self.declare_parameter('workspace_x_min', 0.20)
        self.declare_parameter('workspace_x_max', 0.80)
        self.declare_parameter('workspace_y_min', -0.40)
        self.declare_parameter('workspace_y_max', 0.40)
        self.declare_parameter('workspace_z_min', 0.01)
        self.declare_parameter('workspace_z_max', 0.60)
        self.declare_parameter('object_entity_name', 'target_obj')
        self.declare_parameter('object_init_x', 0.65)
        self.declare_parameter('object_init_y', 0.0)
        self.declare_parameter('object_init_z', 0.05)

        self._approach_offset = self.get_parameter('approach_offset').get_parameter_value().double_value
        self._lift_height = self.get_parameter('lift_height').get_parameter_value().double_value
        self._default_min_score = self.get_parameter('default_min_score').get_parameter_value().double_value
        self._default_max_attempts = self.get_parameter('default_max_attempts').get_parameter_value().integer_value
        self._object_entity_name = self.get_parameter('object_entity_name').get_parameter_value().string_value
        self._object_init_x = self.get_parameter('object_init_x').get_parameter_value().double_value
        self._object_init_y = self.get_parameter('object_init_y').get_parameter_value().double_value
        self._object_init_z = self.get_parameter('object_init_z').get_parameter_value().double_value
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
        self._arm.max_velocity = 0.5
        self._arm.max_acceleration = 0.5
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

        self._current_grasp_pub = self.create_publisher(MarkerArray, '/icgnet/current_grasp_marker', 1)

        self._set_entity_client = self.create_client(
            SetEntityState, '/set_entity_state', callback_group=cb_svc
        )

        self.create_service(ExecuteGrasp, '/icgnet/execute_grasp', self._execute_grasp_cb, callback_group=cb_svc)

        self.get_logger().info('GraspExecutorNode ready.')

    def _grasps_cb(self, msg: GraspArray):
        with self._grasps_lock:
            self._latest_grasps = msg

    def _reset_scene(self):
        self.get_logger().info('[RESET] Opening gripper...')
        self._gripper.open()
        self._gripper.wait_until_executed()

        self.get_logger().info('[RESET] Moving arm to home...')
        self._arm.move_to_configuration(
            joint_positions=HOME_JOINT_POSITIONS,
            joint_names=robot.joint_names(),
        )
        ok = self._arm.wait_until_executed()
        if ok:
            self.get_logger().info('[RESET] Arm at home.')
        else:
            self.get_logger().warn('[RESET] Arm home move failed (continuing anyway).')

        if self._set_entity_client.wait_for_service(timeout_sec=2.0):
            req = SetEntityState.Request()
            req.state.name = self._object_entity_name
            req.state.pose.position.x = self._object_init_x
            req.state.pose.position.y = self._object_init_y
            req.state.pose.position.z = self._object_init_z
            req.state.pose.orientation.w = 1.0
            req.state.reference_frame = 'world'
            fut = self._set_entity_client.call_async(req)
            while not fut.done():
                time.sleep(0.05)
            if fut.result().success:
                self.get_logger().info(
                    f'[RESET] Object "{self._object_entity_name}" reset to '
                    f'[{self._object_init_x}, {self._object_init_y}, {self._object_init_z}]'
                )
            else:
                self.get_logger().warn('[RESET] Object reset service returned failure.')
        else:
            self.get_logger().warn('[RESET] /set_entity_state service not available — object not reset.')

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
        self.get_logger().info(
            f"[SELECT] {len(candidates)} candidates:"
        )
        for i, g in enumerate(candidates):
            q = g.pose.orientation
            approach = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()[:, 2]
            angle_from_vertical = float(np.degrees(np.arccos(np.clip(-approach[2], -1.0, 1.0))))
            p = g.pose.position
            self.get_logger().info(
                f"  [{i+1}] score={g.score:.2f} inst={g.instance_id} cls={g.semantic_class} "
                f"pos=[{p.x:.3f},{p.y:.3f},{p.z:.3f}] "
                f"approach=[{approach[0]:.3f},{approach[1]:.3f},{approach[2]:.3f}] "
                f"angle_from_vertical={angle_from_vertical:.1f}° width={g.width:.3f}"
            )

        self.get_logger().info('[RESET] Resetting scene before first attempt...')
        self._reset_scene()

        for i, g in enumerate(candidates):
            p = g.pose.position
            self.get_logger().info(
                f"{'='*60}\n"
                f"[ATTEMPT {i+1}/{len(candidates)}] score={g.score:.2f} "
                f"inst={g.instance_id} cls={g.semantic_class} "
                f"pos=[{p.x:.3f},{p.y:.3f},{p.z:.3f}]"
            )
            self._publish_current_grasp_marker(g)
            if self._execute_single_grasp(g):
                self._clear_current_grasp_marker()
                res.success = True
                res.grasps_attempted = i + 1
                res.message = f"Grasp succeeded on attempt {i+1}"
                self.get_logger().info(
                    f"[SUCCESS] Grasp completed on attempt {i+1}/{len(candidates)}"
                )
                return res
            self.get_logger().warn(
                f"[ATTEMPT {i+1}/{len(candidates)}] FAILED — "
                f"{'resetting scene and trying next candidate' if i + 1 < len(candidates) else 'no more candidates'}"
            )
            if i + 1 < len(candidates):
                self._reset_scene()

        self._clear_current_grasp_marker()
        res.success = False
        res.grasps_attempted = len(candidates)
        res.message = f"All {len(candidates)} grasp attempts failed"
        return res

    def _filter_grasps(self, grasps, target: str, min_score: float) -> list:
        filtered = []
        n_total = len(grasps)
        n_score = n_ws = n_width = n_target = 0
        for g in grasps:
            p = g.pose.position
            if g.score < min_score:
                n_score += 1
                continue
            if g.width > 0.08:
                n_width += 1
                continue
            if not (self._ws['x'][0] <= p.x <= self._ws['x'][1] and
                    self._ws['y'][0] <= p.y <= self._ws['y'][1] and
                    self._ws['z'][0] <= p.z <= self._ws['z'][1]):
                n_ws += 1
                continue
            if not self._matches_target(g, target):
                n_target += 1
                continue
            filtered.append(g)

        self.get_logger().info(
            f"[FILTER] total={n_total} → kept={len(filtered)} | "
            f"rejected: score={n_score} width={n_width} workspace={n_ws} target={n_target}"
        )
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

    def _publish_current_grasp_marker(self, g):
        pos = np.array([g.pose.position.x, g.pose.position.y, g.pose.position.z])
        q = g.pose.orientation
        R = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        w = float(np.clip(g.width, 0.02, 0.08))
        now = self.get_clock().now().to_msg()

        ma = MarkerArray()
        clear = Marker()
        clear.header.frame_id = 'world'
        clear.header.stamp = now
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = now
        m.ns = 'current_grasp'
        m.id = 0
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.scale.x = 0.006
        m.lifetime = rclpy.duration.Duration(seconds=60).to_msg()

        color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0)  # cyan = active attempt
        for start, end in _gripper_points_world(pos, R, w):
            m.points.append(Point(x=float(start[0]), y=float(start[1]), z=float(start[2])))
            m.points.append(Point(x=float(end[0]),   y=float(end[1]),   z=float(end[2])))
            m.colors.append(color)
            m.colors.append(color)

        ma.markers.append(m)
        self._current_grasp_pub.publish(ma)

    def _clear_current_grasp_marker(self):
        ma = MarkerArray()
        clear = Marker()
        clear.header.frame_id = 'world'
        clear.header.stamp = self.get_clock().now().to_msg()
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)
        self._current_grasp_pub.publish(ma)

    def _execute_single_grasp(self, g) -> bool:
        pos = np.array([g.pose.position.x, g.pose.position.y, g.pose.position.z])
        q = g.pose.orientation
        quat_xyzw = [q.x, q.y, q.z, q.w]
        approach = Rotation.from_quat(quat_xyzw).as_matrix()[:, 2]  # z-col = approach axis

        pre_pos = (pos - self._approach_offset * approach).tolist()
        lift_pos = (pos + np.array([0.0, 0.0, self._lift_height])).tolist()
        angle_deg = float(np.degrees(np.arccos(np.clip(-approach[2], -1.0, 1.0))))

        self.get_logger().info(
            f"[PLAN] grasp_tcp = [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]\n"
            f"       pre_pos  = [{pre_pos[0]:.4f}, {pre_pos[1]:.4f}, {pre_pos[2]:.4f}]\n"
            f"       lift_pos = [{lift_pos[0]:.4f}, {lift_pos[1]:.4f}, {lift_pos[2]:.4f}]\n"
            f"       approach = [{approach[0]:.4f}, {approach[1]:.4f}, {approach[2]:.4f}]"
            f"  ({angle_deg:.1f}° from vertical)\n"
            f"       width    = {g.width:.4f} m"
        )

        # ── Step 1: pre-grasp ──────────────────────────────────────────────────
        self.get_logger().info(
            f"[STEP 1/4] PRE-GRASP → [{pre_pos[0]:.3f}, {pre_pos[1]:.3f}, {pre_pos[2]:.3f}]"
        )
        t0 = time.time()
        self._arm.move_to_pose(position=pre_pos, quat_xyzw=quat_xyzw)
        ok = self._arm.wait_until_executed()
        dt = time.time() - t0
        if not ok:
            self.get_logger().warn(
                f"[STEP 1/4] PRE-GRASP FAILED in {dt:.2f}s — aborting this candidate"
            )
            return False
        self.get_logger().info(f"[STEP 1/4] Pre-grasp reached in {dt:.2f}s")

        # ── Step 2: slow final approach to grasp position ─────────────────────
        self._arm.max_velocity = 0.1
        self._arm.max_acceleration = 0.1
        self.get_logger().info(
            f"[STEP 2/4] APPROACH → [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]"
        )
        t0 = time.time()
        self._arm.move_to_pose(position=pos.tolist(), quat_xyzw=quat_xyzw)
        ok = self._arm.wait_until_executed()
        self._arm.max_velocity = 0.5
        self._arm.max_acceleration = 0.5
        dt = time.time() - t0
        if not ok:
            self.get_logger().warn(
                f"[STEP 2/4] APPROACH FAILED in {dt:.2f}s — aborting this candidate"
            )
            return False
        self.get_logger().info(f"[STEP 2/4] Approach reached in {dt:.2f}s")

        # ── Step 3: close gripper ─────────────────────────────────────────────
        self.get_logger().info("[STEP 3/4] CLOSING GRIPPER <<<")
        self._gripper.close()
        self._gripper.wait_until_executed()
        self.get_logger().info("[STEP 3/4] Gripper closed")

        # ── Step 4: lift ──────────────────────────────────────────────────────
        self.get_logger().info(
            f"[STEP 4/4] LIFT → [{lift_pos[0]:.3f}, {lift_pos[1]:.3f}, {lift_pos[2]:.3f}]"
        )
        t0 = time.time()
        self._arm.move_to_pose(position=lift_pos, quat_xyzw=quat_xyzw)
        self._arm.wait_until_executed()
        self.get_logger().info(f"[STEP 4/4] Lift done in {time.time()-t0:.2f}s")

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
