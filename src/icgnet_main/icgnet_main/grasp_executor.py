import time
from threading import Lock, Thread

import numpy as np
import rclpy
import rclpy.duration
from geometry_msgs.msg import Point, Pose
from shape_msgs.msg import SolidPrimitive
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from moveit_msgs.msg import (
    CollisionObject, PlanningScene, PlanningSceneComponents,
)
from moveit_msgs.srv import GetPlanningScene
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
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
from icgnet_main.pointcloud_utils import gripper_keypoints_world

# Compact home configuration matching the URDF initial_value parameters
HOME_JOINT_POSITIONS = [0.0, -1.0, 0.0, -2.5, 0.0, 2.0, 0.785]

SEMANTIC_CLASSES = {
    'mug': 0, 'box': 1, 'can': 2, 'bottle': 3,
    'cylindric': 4, 'ball': 5, 'other': 6,
}
CLASS_NAMES = {v: k for k, v in SEMANTIC_CLASSES.items()}



class GraspExecutorNode(Node):
    def __init__(self):
        super().__init__('grasp_executor_node')

        self.declare_parameter('rich_grasp_topic', '/icgnet/grasps_rich')
        self.declare_parameter('compute_grasp_service', '/icgnet/compute_grasps')
        self.declare_parameter('default_min_score', 0.5)
        self.declare_parameter('default_max_attempts', 5)
        self.declare_parameter('approach_offset', 0.10)
        self.declare_parameter('grasp_forward_offset', 0.045)
        self.declare_parameter('approach_velocity', 0.08)
        self.declare_parameter('approach_acceleration', 0.08)
        self.declare_parameter('pregrasp_settle_time', 0.8)
        self.declare_parameter('lift_velocity', 0.10)
        self.declare_parameter('lift_acceleration', 0.10)
        self.declare_parameter('cartesian_fraction_threshold', 0.92)
        self.declare_parameter('cartesian_max_step_approach', 0.008)
        self.declare_parameter('lift_height', 0.25)
        self.declare_parameter('workspace_x_min', 0.25)
        self.declare_parameter('workspace_x_max', 1.05)
        self.declare_parameter('workspace_y_min', -0.50)
        self.declare_parameter('workspace_y_max', 0.50)
        self.declare_parameter('workspace_z_min', 0.01)
        self.declare_parameter('workspace_z_max', 0.60)
        self.declare_parameter('object_entity_name', 'target_obj')
        self.declare_parameter('object_init_x', 0.65)
        self.declare_parameter('object_init_y', 0.0)
        self.declare_parameter('object_init_z', 0.05)

        self._approach_offset = self.get_parameter('approach_offset').get_parameter_value().double_value
        self._grasp_forward_offset = self.get_parameter('grasp_forward_offset').get_parameter_value().double_value
        self._approach_velocity = self.get_parameter('approach_velocity').get_parameter_value().double_value
        self._approach_acceleration = self.get_parameter('approach_acceleration').get_parameter_value().double_value
        self._pregrasp_settle_time = self.get_parameter('pregrasp_settle_time').get_parameter_value().double_value
        self._lift_velocity = self.get_parameter('lift_velocity').get_parameter_value().double_value
        self._lift_acceleration = self.get_parameter('lift_acceleration').get_parameter_value().double_value
        self._cartesian_fraction_threshold = self.get_parameter('cartesian_fraction_threshold').get_parameter_value().double_value
        self._cartesian_max_step_approach = self.get_parameter('cartesian_max_step_approach').get_parameter_value().double_value
        self._lift_height = self.get_parameter('lift_height').get_parameter_value().double_value
        self._default_min_score = self.get_parameter('default_min_score').get_parameter_value().double_value
        self._default_max_attempts = self.get_parameter('default_max_attempts').get_parameter_value().integer_value
        self._object_entity_name = self.get_parameter('object_entity_name').get_parameter_value().string_value
        self._object_init_x = self.get_parameter('object_init_x').get_parameter_value().double_value
        self._object_init_y = self.get_parameter('object_init_y').get_parameter_value().double_value
        self._object_init_z = self.get_parameter('object_init_z').get_parameter_value().double_value
        self._workspace_bounds = {
            'x': (self.get_parameter('workspace_x_min').get_parameter_value().double_value,
                  self.get_parameter('workspace_x_max').get_parameter_value().double_value),
            'y': (self.get_parameter('workspace_y_min').get_parameter_value().double_value,
                  self.get_parameter('workspace_y_max').get_parameter_value().double_value),
            'z': (self.get_parameter('workspace_z_min').get_parameter_value().double_value,
                  self.get_parameter('workspace_z_max').get_parameter_value().double_value),
        }

        self.declare_parameter('use_collision_scene', True)
        self.declare_parameter('attach_weight', 0.35)

        # ── Pick-and-place ────────────────────────────────────────────────────
        self.declare_parameter('place_x', 0.45)
        self.declare_parameter('place_y', -0.30)
        self.declare_parameter('place_release_z', 0.24)
        self.declare_parameter('place_pre_offset', 0.12)
        self.declare_parameter('bin_footprint', 0.18)
        self.declare_parameter('bin_rim_height', 0.08)
        self.declare_parameter('transport_velocity', 0.10)
        self.declare_parameter('transport_acceleration', 0.10)
        self.declare_parameter('transport_clear_z', 0.45)
        self.declare_parameter('safe_retract_height', 0.15)
        self.declare_parameter('joint_state_fresh_timeout', 2.0)
        self.declare_parameter('acm_allowed_links',
                               ['panda_leftfinger', 'panda_rightfinger', 'panda_hand'])
        self.declare_parameter('collision_id_prefix', 'icgnet_inst_')
        self.declare_parameter('finger_safety_margin', 0.01)
        self.declare_parameter('max_finger_pos', 0.04)
        self.declare_parameter('release_settle_states', 20)
        self.declare_parameter('min_finger_gap', 0.005)
        self.declare_parameter('max_finger_gap', 0.036)
        self.declare_parameter('executor_threads', 4)
        self.declare_parameter('max_grasp_width', 0.08)
        self.declare_parameter('arm_default_velocity', 0.5)
        self.declare_parameter('arm_default_acceleration', 0.5)
        self.declare_parameter('allowed_planning_time', 5.0)
        self.declare_parameter('num_planning_attempts', 10)
        self.declare_parameter('inference_timeout', 120.0)
        self.declare_parameter('max_approach_angle_deg', 90.0)
        self.declare_parameter('min_approach_angle_deg', 0.0)

        self._use_collision_scene = self.get_parameter('use_collision_scene').get_parameter_value().bool_value
        self._attach_weight = self.get_parameter('attach_weight').get_parameter_value().double_value
        self._place_x = self.get_parameter('place_x').get_parameter_value().double_value
        self._place_y = self.get_parameter('place_y').get_parameter_value().double_value
        self._place_release_z = self.get_parameter('place_release_z').get_parameter_value().double_value
        self._place_pre_offset = self.get_parameter('place_pre_offset').get_parameter_value().double_value
        self._bin_footprint = self.get_parameter('bin_footprint').get_parameter_value().double_value
        self._bin_rim_height = self.get_parameter('bin_rim_height').get_parameter_value().double_value
        self._transport_velocity = self.get_parameter('transport_velocity').get_parameter_value().double_value
        self._transport_acceleration = self.get_parameter('transport_acceleration').get_parameter_value().double_value
        self._transport_clear_z = self.get_parameter('transport_clear_z').get_parameter_value().double_value
        self._safe_retract_height = self.get_parameter('safe_retract_height').get_parameter_value().double_value
        self._joint_state_fresh_timeout = self.get_parameter('joint_state_fresh_timeout').get_parameter_value().double_value
        self._acm_allowed_links = self.get_parameter('acm_allowed_links').get_parameter_value().string_array_value
        self._collision_id_prefix = self.get_parameter('collision_id_prefix').get_parameter_value().string_value
        self._finger_safety_margin = self.get_parameter('finger_safety_margin').get_parameter_value().double_value
        self._max_finger_pos = self.get_parameter('max_finger_pos').get_parameter_value().double_value
        self._release_settle_states = self.get_parameter('release_settle_states').get_parameter_value().integer_value
        self._min_finger_gap = self.get_parameter('min_finger_gap').get_parameter_value().double_value
        self._max_finger_gap = self.get_parameter('max_finger_gap').get_parameter_value().double_value
        self._max_grasp_width = self.get_parameter('max_grasp_width').get_parameter_value().double_value
        self._arm_default_velocity = self.get_parameter('arm_default_velocity').get_parameter_value().double_value
        self._arm_default_acceleration = self.get_parameter('arm_default_acceleration').get_parameter_value().double_value
        self._allowed_planning_time = self.get_parameter('allowed_planning_time').get_parameter_value().double_value
        self._num_planning_attempts = self.get_parameter('num_planning_attempts').get_parameter_value().integer_value
        self._inference_timeout = self.get_parameter('inference_timeout').get_parameter_value().double_value
        self._max_approach_angle_deg = self.get_parameter('max_approach_angle_deg').get_parameter_value().double_value
        self._min_approach_angle_deg = self.get_parameter('min_approach_angle_deg').get_parameter_value().double_value

        # Callback groups: MutuallyExclusive for the grasp service and planning-scene clients
        # to prevent deadlocks in MultiThreadedExecutor (ROS2 guideline).
        cb_grasp_exec = MutuallyExclusiveCallbackGroup()
        cb_arm = ReentrantCallbackGroup()
        cb_gripper = ReentrantCallbackGroup()
        cb_svc = MutuallyExclusiveCallbackGroup()
        cb_sub = ReentrantCallbackGroup()

        self._arm = MoveIt2(
            node=self,
            joint_names=robot.joint_names(),
            base_link_name=robot.base_link_name(),
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=cb_arm,
        )
        self._arm.max_velocity = self._arm_default_velocity
        self._arm.max_acceleration = self._arm_default_acceleration
        self._arm.allowed_planning_time = self._allowed_planning_time
        self._arm.num_planning_attempts = self._num_planning_attempts
        self._arm.cartesian_jump_threshold = 0.0

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
            SetEntityPose, '/world/icgnet_world/set_pose', callback_group=cb_svc
        )

        self._get_scene_client = self.create_client(
            GetPlanningScene, '/get_planning_scene', callback_group=cb_svc
        )

        self._co_pub = self.create_publisher(CollisionObject, '/collision_object', 10)

        # Tracks the CO that is currently being managed so _reset_scene() can
        # remove it before planning (arm may be inside the CO after a failed attempt).
        self._active_co_id: str | None = None
        self._active_co: CollisionObject | None = None

        self.create_service(ExecuteGrasp, '/icgnet/execute_grasp', self._execute_grasp_cb, callback_group=cb_grasp_exec)

        # Published lazily on the first execute_grasp call, when move_group is ready.
        self._bin_co_published = False

        self.get_logger().info('GraspExecutorNode ready.')

    def _grasps_cb(self, msg: GraspArray):
        with self._grasps_lock:
            self._latest_grasps = msg

    def _reset_scene(self, teleport_object: bool = True):
        # Remove active CO from the planning scene BEFORE any motion planning.
        # After a failed attempt the arm may be inside or adjacent to the CO volume,
        # making the current state appear in collision → INVALID_MOTION_PLAN on every
        # subsequent call.  CO is re-added after the arm is safely at home.
        if self._use_collision_scene and self._active_co_id is not None:
            if self._active_co is None:
                # CO may still be in the scene (e.g. Step-1 failure before fetch).
                self._active_co = self._fetch_co_from_scene(self._active_co_id)
            self._remove_co_from_scene(self._active_co_id)
            time.sleep(0.15)  # let move_group process REMOVE before planning

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

        # Teleport only between retry attempts. Before attempt 1 the object is already
        # at the position seen by inference — teleporting would move it away from the
        # predicted grasp poses and cause misses.
        if teleport_object:
            if self._set_entity_client.wait_for_service(timeout_sec=2.0):
                req = SetEntityPose.Request()
                req.entity.name = self._object_entity_name
                req.entity.type = Entity.MODEL
                req.pose.position.x = self._object_init_x
                req.pose.position.y = self._object_init_y
                req.pose.position.z = self._object_init_z
                req.pose.orientation.w = 1.0
                fut = self._set_entity_client.call_async(req)
                if not self._wait_for_future(fut):
                    self.get_logger().warn('[RESET] Object reset timed out.')
                elif fut.result().success:
                    self.get_logger().info(
                        f'[RESET] Object "{self._object_entity_name}" reset to '
                        f'[{self._object_init_x}, {self._object_init_y}, {self._object_init_z}]'
                    )
                else:
                    self.get_logger().warn('[RESET] Object reset service returned failure.')
            else:
                self.get_logger().warn('[RESET] /world/icgnet_world/set_pose not available — object not reset.')

        # Re-add CO so the next attempt's Step-1 planning can avoid the object.
        if self._use_collision_scene and self._active_co is not None:
            self._readd_co_to_scene(self._active_co)
            self.get_logger().info(f"[RESET] Re-added CO '{self._active_co_id}' to scene.")

    def _execute_grasp_cb(self, req: ExecuteGrasp.Request, res: ExecuteGrasp.Response):
        target = req.target.strip() if req.target else 'any'
        min_score = req.min_score if req.min_score > 0.0 else self._default_min_score
        max_attempts = req.max_attempts if req.max_attempts > 0 else self._default_max_attempts
        skip_place = req.skip_place

        self.get_logger().info(
            f"ExecuteGrasp: target='{target}' min_score={min_score:.2f} max_attempts={max_attempts} "
            f"skip_place={skip_place}"
        )

        # Ensure the drop-bin collision object is in the planning scene.
        if not self._bin_co_published:
            self._add_bin_collision_object()
            self._bin_co_published = True

        # Clear stale grasps so we know the next GraspArray is fresh
        with self._grasps_lock:
            self._latest_grasps = None

        if not self._compute_client.wait_for_service(timeout_sec=5.0):
            res.success = False
            res.message = f"Service '{self._compute_client.srv_name}' not available"
            return res

        future = self._compute_client.call_async(Trigger.Request())
        if not self._wait_for_future(future, timeout=self._inference_timeout):
            res.success = False
            res.message = f"ICGNet inference timed out after {self._inference_timeout:.0f}s"
            return res
        if not future.result().success:
            res.success = False
            res.message = f"ICGNet inference failed: {future.result().message}"
            return res

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
                f"  [{i+1}] score={g.score:.2f} inst={g.instance_id} cls={g.semantic_class}({CLASS_NAMES.get(g.semantic_class,'?')}) "
                f"pos=[{p.x:.3f},{p.y:.3f},{p.z:.3f}] "
                f"approach=[{approach[0]:.3f},{approach[1]:.3f},{approach[2]:.3f}] "
                f"angle_from_vertical={angle_from_vertical:.1f}° width={g.width:.3f}"
            )

        self.get_logger().info('[RESET] Resetting arm/gripper before first attempt (object stays at inference position)...')
        self._reset_scene(teleport_object=False)

        for i, g in enumerate(candidates):
            p = g.pose.position
            self.get_logger().info(
                f"{'='*60}\n"
                f"[ATTEMPT {i+1}/{len(candidates)}] score={g.score:.2f} "
                f"inst={g.instance_id} cls={g.semantic_class}({CLASS_NAMES.get(g.semantic_class,'?')}) "
                f"pos=[{p.x:.3f},{p.y:.3f},{p.z:.3f}]"
            )
            if self._execute_single_grasp(g, skip_place=skip_place):
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
        n_score = n_ws = n_width = n_target = n_angle = 0
        for g in grasps:
            p = g.pose.position
            if g.score < min_score:
                n_score += 1
                continue
            if g.width > self._max_grasp_width:
                n_width += 1
                continue
            if not (self._workspace_bounds['x'][0] <= p.x <= self._workspace_bounds['x'][1] and
                    self._workspace_bounds['y'][0] <= p.y <= self._workspace_bounds['y'][1] and
                    self._workspace_bounds['z'][0] <= p.z <= self._workspace_bounds['z'][1]):
                n_ws += 1
                continue
            if not self._matches_target(g, target):
                n_target += 1
                continue
            q = g.pose.orientation
            approach = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()[:, 2]
            angle_deg = float(np.degrees(np.arccos(np.clip(-approach[2], -1.0, 1.0))))
            if angle_deg > self._max_approach_angle_deg or angle_deg < self._min_approach_angle_deg:
                n_angle += 1
                continue
            filtered.append(g)

        filtered.sort(key=lambda g: g.score, reverse=True)
        score_range = (
            f"[{filtered[-1].score:.2f}–{filtered[0].score:.2f}]"
            if filtered else "—"
        )
        self.get_logger().info(
            f"[FILTER] total={n_total} → kept={len(filtered)} (min_score≥{min_score:.2f}, "
            f"scores={score_range}, angle=[{self._min_approach_angle_deg:.0f}°–{self._max_approach_angle_deg:.0f}°]) | "
            f"rejected: score={n_score} width={n_width} workspace={n_ws} target={n_target} angle={n_angle}"
        )
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

    def _publish_current_grasp_marker(self, g, contact_pos: np.ndarray):
        pos = np.array([g.pose.position.x, g.pose.position.y, g.pose.position.z])
        q = g.pose.orientation
        R = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        w = float(np.clip(g.width, 0.02, 0.08))
        now = self.get_clock().now().to_msg()
        lifetime = rclpy.duration.Duration(seconds=60).to_msg()

        ma = MarkerArray()
        clear = Marker()
        clear.header.frame_id = 'world'
        clear.header.stamp = now
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        # Cyan: ICGNet predicted TCP (backed off 0.045 m from contact surface)
        m_tcp = Marker()
        m_tcp.header.frame_id = 'world'
        m_tcp.header.stamp = now
        m_tcp.ns = 'current_grasp'
        m_tcp.id = 0
        m_tcp.type = Marker.LINE_LIST
        m_tcp.action = Marker.ADD
        m_tcp.scale.x = 0.005
        m_tcp.lifetime = lifetime
        color_cyan = ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.7)
        for start, end in gripper_keypoints_world(pos, R, w):
            m_tcp.points.append(Point(x=float(start[0]), y=float(start[1]), z=float(start[2])))
            m_tcp.points.append(Point(x=float(end[0]),   y=float(end[1]),   z=float(end[2])))
            m_tcp.colors.append(color_cyan)
            m_tcp.colors.append(color_cyan)
        ma.markers.append(m_tcp)

        # Green: actual grasp target (contact_pos = pos + forward_offset * approach)
        m_contact = Marker()
        m_contact.header.frame_id = 'world'
        m_contact.header.stamp = now
        m_contact.ns = 'current_grasp'
        m_contact.id = 1
        m_contact.type = Marker.LINE_LIST
        m_contact.action = Marker.ADD
        m_contact.scale.x = 0.007
        m_contact.lifetime = lifetime
        color_green = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        for start, end in gripper_keypoints_world(contact_pos, R, w):
            m_contact.points.append(Point(x=float(start[0]), y=float(start[1]), z=float(start[2])))
            m_contact.points.append(Point(x=float(end[0]),   y=float(end[1]),   z=float(end[2])))
            m_contact.colors.append(color_green)
            m_contact.colors.append(color_green)
        ma.markers.append(m_contact)

        self._current_grasp_pub.publish(ma)

    def _clear_current_grasp_marker(self):
        ma = MarkerArray()
        clear = Marker()
        clear.header.frame_id = 'world'
        clear.header.stamp = self.get_clock().now().to_msg()
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)
        self._current_grasp_pub.publish(ma)

    def _wait_for_future(self, future, timeout: float = 5.0) -> bool:
        deadline = time.time() + timeout
        while not future.done():
            if time.time() > deadline:
                return False
            time.sleep(0.02)
        return True

    def _fetch_co_from_scene(self, co_id: str):
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        fut = self._get_scene_client.call_async(req)
        if not self._wait_for_future(fut):
            self.get_logger().warn(f"[CO] GetPlanningScene timed out while fetching '{co_id}'")
            return None
        for co in fut.result().scene.world.collision_objects:
            if co.id == co_id:
                return co
        self.get_logger().warn(f"[CO] '{co_id}' not found in planning scene")
        return None

    def _remove_co_from_scene(self, co_id: str):
        msg = CollisionObject()
        msg.id = co_id
        msg.header.frame_id = 'world'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.operation = CollisionObject.REMOVE
        self._co_pub.publish(msg)

    def _readd_co_to_scene(self, co: CollisionObject | None):
        if co is None:
            return
        co.operation = CollisionObject.ADD
        co.header.stamp = self.get_clock().now().to_msg()
        self._co_pub.publish(co)
        time.sleep(0.15)  # let move_group process ADD before planning

    def _abort_grasp(self, target_co_id: str) -> bool:
        self._arm.max_velocity = self._arm_default_velocity
        self._arm.max_acceleration = self._arm_default_acceleration
        return False

    def _execute_single_grasp(self, g, skip_place: bool = False) -> bool:
        # Guarantee default velocity at every attempt start, regardless of prior state.
        self._arm.max_velocity = self._arm_default_velocity
        self._arm.max_acceleration = self._arm_default_acceleration

        pos = np.array([g.pose.position.x, g.pose.position.y, g.pose.position.z])
        q = g.pose.orientation
        quat_xyzw = [q.x, q.y, q.z, q.w]
        approach = Rotation.from_quat(quat_xyzw).as_matrix()[:, 2]  # z-col = approach axis

        contact_pos = pos + self._grasp_forward_offset * approach
        pre_pos = (pos - self._approach_offset * approach).tolist()
        lift_pos = (contact_pos + np.array([0.0, 0.0, self._lift_height])).tolist()
        angle_deg = float(np.degrees(np.arccos(np.clip(-approach[2], -1.0, 1.0))))

        self._publish_current_grasp_marker(g, contact_pos)

        self.get_logger().info(
            f"[PLAN] score={g.score:.3f}  inst={g.instance_id}  cls={g.semantic_class}({CLASS_NAMES.get(g.semantic_class,'?')})  width={g.width:.4f}m\n"
            f"       icgnet_tcp  = [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]\n"
            f"       contact_pos = [{contact_pos[0]:.4f}, {contact_pos[1]:.4f}, {contact_pos[2]:.4f}]"
            f"  (forward_offset={self._grasp_forward_offset*100:.1f}cm)\n"
            f"       pre_pos     = [{pre_pos[0]:.4f}, {pre_pos[1]:.4f}, {pre_pos[2]:.4f}]\n"
            f"       lift_pos    = [{lift_pos[0]:.4f}, {lift_pos[1]:.4f}, {lift_pos[2]:.4f}]\n"
            f"       approach    = [{approach[0]:.4f}, {approach[1]:.4f}, {approach[2]:.4f}]"
            f"  ({angle_deg:.1f}° from vertical)"
        )

        target_co_id = f"{self._collision_id_prefix}{g.instance_id}"
        self._active_co_id = target_co_id
        self._active_co = None  # will be set after CO is fetched post-Step-1

        per_finger_pos = min(g.width / 2.0 + self._finger_safety_margin, self._max_finger_pos)
        actual_opening = 2.0 * per_finger_pos
        icgnet_opening = g.width
        width_note = (
            f" [CAPPED at max_finger_pos={self._max_finger_pos*1000:.0f}mm/side]"
            if per_finger_pos == self._max_finger_pos else ""
        )
        if abs(actual_opening - icgnet_opening) > 0.002:
            self.get_logger().warn(
                f"[GRIPPER] ICGNet width={icgnet_opening*1000:.1f}mm → "
                f"pre-grasp opening={actual_opening*1000:.1f}mm (per finger={per_finger_pos*1000:.1f}mm){width_note}"
            )
        else:
            self.get_logger().info(
                f"[GRIPPER] pre-grasp opening={actual_opening*1000:.1f}mm "
                f"(ICGNet={icgnet_opening*1000:.1f}mm, per finger={per_finger_pos*1000:.1f}mm){width_note}"
            )
        self._gripper.move_to_position(per_finger_pos)
        self._gripper.wait_until_executed()

        # ── Step 0: update collision scene ───────────────────────────────────
        if self._use_collision_scene:
            self._arm.update_planning_scene()

        # ── Step 1/5: pre-grasp (joint-space, default velocity) ───────────────
        self.get_logger().info(
            f"[STEP 1/5] PRE-GRASP → [{pre_pos[0]:.3f}, {pre_pos[1]:.3f}, {pre_pos[2]:.3f}]"
        )
        t0 = time.time()
        self._arm.move_to_pose(position=pre_pos, quat_xyzw=quat_xyzw)
        ok = self._arm.wait_until_executed()
        dt = time.time() - t0
        if not ok:
            self.get_logger().warn(
                f"[STEP 1/5] PRE-GRASP FAILED in {dt:.2f}s — aborting this candidate"
            )
            return self._abort_grasp(target_co_id)
        self.get_logger().info(f"[STEP 1/5] Pre-grasp reached in {dt:.2f}s")
        time.sleep(self._pregrasp_settle_time)

        self._arm.max_velocity = self._approach_velocity
        self._arm.max_acceleration = self._approach_acceleration

        # Remove CO from scene so the Cartesian approach path is unblocked.
        # The CO is fetched first so it can be re-added before attach in Step 3b.
        # Pre-grasp (Step 1) ran with CO in scene, so joint-space planning correctly
        # avoided it; only now (outside the object) do we remove it.
        _target_co = None
        if self._use_collision_scene:
            _target_co = self._fetch_co_from_scene(target_co_id)
            self._active_co = _target_co  # save so _reset_scene() can re-add after recovery
            self._remove_co_from_scene(target_co_id)
            self.get_logger().info(f"[CO] Removed '{target_co_id}' from scene for approach")

        # ── Step 2/5: Cartesian approach → contact surface ───────────────────────
        # contact_pos = pos + grasp_forward_offset * approach
        # where pos is ICGNet TCP (0.045 m behind contact along approach),
        # so contact_pos = exactly the predicted contact point on the object surface.
        self.get_logger().info(
            f"[STEP 2/5] CARTESIAN APPROACH → [{contact_pos[0]:.3f}, {contact_pos[1]:.3f}, {contact_pos[2]:.3f}]"
            f"  (vel={self._approach_velocity})"
        )
        t0 = time.time()
        self._arm.move_to_pose(
            position=contact_pos.tolist(), quat_xyzw=quat_xyzw,
            cartesian=True, cartesian_max_step=self._cartesian_max_step_approach,
            cartesian_fraction_threshold=self._cartesian_fraction_threshold,
        )
        ok = self._arm.wait_until_executed()
        dt = time.time() - t0
        if not ok:
            self.get_logger().warn(
                f"[STEP 2/5] APPROACH FAILED in {dt:.2f}s — aborting this candidate"
            )
            # CO stays absent: _reset_scene() will re-add it after moving home.
            return self._abort_grasp(target_co_id)
        self.get_logger().info(f"[STEP 2/5] Contact surface reached in {dt:.2f}s")

        # ── Step 3/5: close gripper + verify grasp ────────────────────────────
        self.get_logger().info("[STEP 3/5] CLOSING GRIPPER")
        self._gripper.close()
        self._gripper.wait_until_executed()
        # Demand a fresh joint_state: wall-clock sleep is meaningless at low RTF (e.g. 0.03).
        # At RTF 0.03 the broadcaster runs at 50 Hz sim ≈ 1.5 msg/s wall; 0.5s sleep
        # would read a state from before the gripper command settled.
        finger_gap = self._read_finger_gap_fresh(self._joint_state_fresh_timeout)

        # OPEN=0.04m, CLOSED=0.0m per finger; threshold 5mm means jaw gap > 10mm total.
        if finger_gap < self._min_finger_gap:
            self.get_logger().warn(
                f"[STEP 3/5] Gripper fully closed (gap={finger_gap*1000:.1f}mm < "
                f"{self._min_finger_gap*1000:.0f}mm) — missed object"
            )
            # CO stays absent: _reset_scene() will re-add it after moving home.
            return self._abort_grasp(target_co_id)
        if finger_gap > self._max_finger_gap:
            self.get_logger().warn(
                f"[STEP 3/5] Gripper barely closed (gap={finger_gap*1000:.1f}mm > "
                f"{self._max_finger_gap*1000:.0f}mm) — object tipped or controller aborted"
            )
            # CO stays absent: _reset_scene() will re-add it after moving home.
            return self._abort_grasp(target_co_id)
        self.get_logger().info(
            f"[STEP 3/5] Object confirmed between fingers "
            f"(gap={finger_gap*1000:.1f}mm, range [{self._min_finger_gap*1000:.0f}–{self._max_finger_gap*1000:.0f}mm])"
        )

        # ── Step 3b: re-add CO then attach to gripper for collision-aware lift ──
        if self._use_collision_scene:
            self._readd_co_to_scene(_target_co)
            try:
                self._arm.attach_collision_object(
                    id=target_co_id,
                    link_name='panda_hand_tcp',
                    touch_links=list(self._acm_allowed_links),
                    weight=self._attach_weight,
                )
                self.get_logger().info(f"[STEP 3b] Attached '{target_co_id}' to panda_hand_tcp")
            except Exception as e:
                self.get_logger().warn(f"[STEP 3b] Attach failed (non-fatal): {e}")

        # ── Step 4/5: Cartesian lift (straight +Z from grasp position) ────────
        # Cartesian path prevents the STATUS_ABORTED that occurs when joint-space
        # planning cannot find an IK solution from the extended grasp configuration.
        self._arm.max_velocity = self._lift_velocity
        self._arm.max_acceleration = self._lift_acceleration
        self.get_logger().info(
            f"[STEP 4/5] CARTESIAN LIFT → [{lift_pos[0]:.3f}, {lift_pos[1]:.3f}, {lift_pos[2]:.3f}]"
        )
        t0 = time.time()
        self._arm.move_to_pose(
            position=lift_pos, quat_xyzw=quat_xyzw,
            cartesian=True, cartesian_max_step=self._cartesian_max_step_approach,
            cartesian_fraction_threshold=self._cartesian_fraction_threshold,
        )
        ok = self._arm.wait_until_executed()
        dt = time.time() - t0
        if not ok:
            self.get_logger().warn(f"[STEP 4/5] LIFT FAILED in {dt:.2f}s")
            if self._use_collision_scene:
                try:
                    self._arm.detach_collision_object(id=target_co_id)
                    self._arm.remove_collision_object(id=target_co_id)
                except Exception as e:
                    self.get_logger().warn(f"[STEP 4 fail] Detach failed: {e}")
            return self._abort_grasp(target_co_id)
        self.get_logger().info(f"[STEP 4/5] Object lifted in {dt:.2f}s")

        # Verify object is still gripped after lift (drop detection).
        # Only lower bound: gap→0 means fingers collapsed on empty air = object dropped.
        # Upper bound removed: valid grasp gap varies by object and oscillates slightly.
        finger_gap_post = self._read_finger_gap_fresh(self._joint_state_fresh_timeout)
        if finger_gap_post < self._min_finger_gap:
            self.get_logger().warn(
                f"[STEP 4/5 POST-CHECK] Object dropped during lift "
                f"(gap={finger_gap_post*1000:.1f}mm < {self._min_finger_gap*1000:.0f}mm)"
            )
            if self._use_collision_scene:
                try:
                    self._arm.detach_collision_object(id=target_co_id)
                    self._arm.remove_collision_object(id=target_co_id)
                except Exception as e:
                    self.get_logger().warn(f"[STEP 4 drop] Detach failed: {e}")
            return self._abort_grasp(target_co_id)

        # ── Steps 5–7 + HOME: transport → lower → release → retract → HOME ─────
        return self._place_object(quat_xyzw, target_co_id, skip_place)


    def _read_finger_gap(self) -> float:
        """Return the maximum finger joint position as a proxy for gripper opening."""
        js = self._arm.joint_state
        gap = 0.0
        if js is not None:
            for fname in robot.gripper_joint_names():
                try:
                    gap = max(gap, js.position[js.name.index(fname)])
                except ValueError:
                    pass
        return gap

    def _read_finger_gap_fresh(self, timeout: float = 2.0) -> float:
        """Wait for a fresh joint_state, then return the finger gap.

        Wall-clock sleeps are unreliable at low RTF: at RTF 0.03 the joint_state
        broadcaster runs at 50 Hz sim ≈ 1.5 msg/s wall, so a 0.5 s sleep may read
        a stale value from before the last gripper command.  This helper resets the
        pymoveit2 freshness flag and waits up to `timeout` seconds for a new message.
        """
        self._arm.reset_new_joint_state_checker()
        deadline = time.time() + timeout
        while not self._arm.new_joint_state_available:
            if time.time() > deadline:
                self.get_logger().warn(
                    f"[GAP] No fresh joint_state in {timeout:.1f}s — reading last available"
                )
                break
            time.sleep(0.01)
        return self._read_finger_gap()

    def _settle_release(self, n_states: int) -> float:
        """Wait for n_states fresh joint_state messages after gripper open.
        RTF-independent: each _read_finger_gap_fresh call waits for one broadcaster tick."""
        gap = 0.0
        for _ in range(n_states):
            gap = self._read_finger_gap_fresh(self._joint_state_fresh_timeout)
        return gap

    def _add_bin_collision_object(self):
        """Publish a solid-box CollisionObject for the drop bin up to rim height.
        The gripper always releases above the rim, so this conservative representation
        blocks MoveIt from planning through the bin structure."""
        co = CollisionObject()
        co.header.frame_id = 'world'
        co.header.stamp = self.get_clock().now().to_msg()
        co.id = 'drop_bin'
        co.operation = CollisionObject.ADD

        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [self._bin_footprint, self._bin_footprint, self._bin_rim_height]
        co.primitives = [prim]

        pose = Pose()
        pose.position.x = self._place_x
        pose.position.y = self._place_y
        pose.position.z = self._bin_rim_height / 2.0
        pose.orientation.w = 1.0
        co.primitive_poses = [pose]

        self._co_pub.publish(co)
        self.get_logger().info(
            f"[BIN CO] Published drop_bin at ({self._place_x:.2f}, {self._place_y:.2f}), "
            f"footprint={self._bin_footprint:.2f}m, rim_h={self._bin_rim_height:.2f}m"
        )

    def _place_object(self, quat_xyzw: list, target_co_id: str, skip_place: bool) -> bool:
        """Steps 5–7 + HOME: three-phase transport → lower → release → retract → HOME.

        Transport strategy (three phases):
          A. Cartesian lift-clear: straight up to transport_clear_z — short, high fraction.
          B. Joint-space transfer: OMPL to place_pre_pos with grasp orientation as IK target.
             OMPL plans freely over the long arc; no orientation-seed IK → no "joint limits" fail.
          C. Cartesian lower: straight down to place_release_z, orientation locked.

        The arm only reorients its wrist AFTER release (HOME joint-space, gripper empty).
        """
        if skip_place:
            # Debug path: old behaviour — go HOME while holding the object.
            self._arm.max_velocity = self._arm_default_velocity
            self._arm.max_acceleration = self._arm_default_acceleration
            self.get_logger().info("[STEP 5/5] SKIP_PLACE → HOME (debug, object held)")
            t0 = time.time()
            self._arm.move_to_configuration(
                joint_positions=HOME_JOINT_POSITIONS, joint_names=robot.joint_names()
            )
            ok = self._arm.wait_until_executed()
            self.get_logger().info(
                f"[STEP 5/5] Home {'reached' if ok else 'FAILED'} in {time.time()-t0:.2f}s"
            )
            if self._use_collision_scene:
                try:
                    self._arm.detach_collision_object(id=target_co_id)
                    self._arm.remove_collision_object(id=target_co_id)
                except Exception as e:
                    self.get_logger().warn(f"[SKIP_PLACE] Detach failed (non-fatal): {e}")
            self._active_co_id = None
            self._active_co = None
            return True

        transfer_pos = [self._place_x, self._place_y, self._transport_clear_z]
        place_release_pos = [self._place_x, self._place_y, self._place_release_z]
        retract_pos = [self._place_x, self._place_y,
                       self._place_release_z + self._safe_retract_height]

        self._arm.max_velocity = self._transport_velocity
        self._arm.max_acceleration = self._transport_acceleration

        def _co_detach_remove():
            if self._use_collision_scene:
                try:
                    self._arm.detach_collision_object(id=target_co_id)
                    self._arm.remove_collision_object(id=target_co_id)
                except Exception as e:
                    self.get_logger().warn(f"[PLACE] Detach/remove failed: {e}")

        # ── Step 5/7: joint-space transfer → above bin ───────────────────────────
        # OMPL plans the long lateral arc freely. Orientation (quat_xyzw) is an IK GOAL
        # on the TARGET, not a path constraint, so pick_ik finds a valid seed without hitting
        # joint limits. Both start (lift, gripper-down) and goal (above bin, gripper-down)
        # share a similar wrist configuration → OMPL finds a smooth path.
        self.get_logger().info(
            f"[STEP 5/7] TRANSFER → {[f'{v:.3f}' for v in transfer_pos]}"
            f"  (joint-space, orient=target, vel={self._transport_velocity})"
        )
        t0 = time.time()
        self._arm.move_to_pose(position=transfer_pos, quat_xyzw=quat_xyzw, cartesian=False)
        ok = self._arm.wait_until_executed()
        if not ok:
            self.get_logger().warn(
                f"[STEP 5/7] TRANSFER FAILED in {time.time()-t0:.2f}s — aborting"
            )
            _co_detach_remove()
            return self._abort_grasp(target_co_id)
        self.get_logger().info(f"[STEP 5/7] Transfer done in {time.time()-t0:.2f}s")

        gap = self._read_finger_gap_fresh(self._joint_state_fresh_timeout)
        if gap < self._min_finger_gap:
            self.get_logger().warn(
                f"[STEP 5/7] Object dropped during transfer "
                f"(gap={gap*1000:.1f}mm < {self._min_finger_gap*1000:.0f}mm) — aborting"
            )
            _co_detach_remove()
            return self._abort_grasp(target_co_id)

        # ── Step 6/7: Cartesian lower → place_release ────────────────────────────
        # Short descent (transport_clear_z → place_release_z ≈ 19cm). Fraction should be
        # high: starting from directly above the release point, same orientation.
        self.get_logger().info(
            f"[STEP 6/7] LOWER → {[f'{v:.3f}' for v in place_release_pos]}"
        )
        t0 = time.time()
        self._arm.move_to_pose(
            position=place_release_pos, quat_xyzw=quat_xyzw,
            cartesian=True,
            cartesian_max_step=self._cartesian_max_step_approach,
            cartesian_fraction_threshold=self._cartesian_fraction_threshold,
        )
        ok = self._arm.wait_until_executed()
        if not ok:
            self.get_logger().warn(
                f"[STEP 6/7] LOWER FAILED in {time.time()-t0:.2f}s — aborting"
            )
            _co_detach_remove()
            return self._abort_grasp(target_co_id)
        self.get_logger().info(f"[STEP 6/7] Release position reached in {time.time()-t0:.2f}s")

        gap = self._read_finger_gap_fresh(self._joint_state_fresh_timeout)
        if gap < self._min_finger_gap:
            self.get_logger().warn(
                f"[STEP 6/7] Object dropped during lower "
                f"(gap={gap*1000:.1f}mm < {self._min_finger_gap*1000:.0f}mm) — aborting"
            )
            _co_detach_remove()
            return self._abort_grasp(target_co_id)

        # ── Step 7/7: open gripper → release object ────────────────────────────
        self.get_logger().info("[STEP 7/7] RELEASING OBJECT")
        self._gripper.open()
        self._gripper.wait_until_executed()
        # Accumulate N fresh joint_state messages so the object has sim-time to fall
        # free before the retract move. RTF-independent: each call waits for one
        # broadcaster tick at 50 Hz sim (~release_settle_states * 20ms sim = 0.4s sim).
        gap_after_open = self._settle_release(self._release_settle_states)
        self.get_logger().info(
            f"[STEP 7/7] Finger gap after open: {gap_after_open*1000:.1f}mm "
            f"(~0=gripper stuck/not open, ~40=fully open)"
        )

        _co_detach_remove()
        if self._use_collision_scene:
            self.get_logger().info(f"[STEP 7/7] Detached '{target_co_id}'")

        self._active_co_id = None
        self._active_co = None

        # ── Retract: Cartesian +Z before going HOME ────────────────────────────
        self.get_logger().info(f"[RETRACT] Cartesian retract → z={retract_pos[2]:.3f}")
        self._arm.move_to_pose(
            position=retract_pos, quat_xyzw=quat_xyzw,
            cartesian=True,
            cartesian_max_step=self._cartesian_max_step_approach,
            cartesian_fraction_threshold=self._cartesian_fraction_threshold,
        )
        ok = self._arm.wait_until_executed()
        if not ok:
            self.get_logger().warn("[RETRACT] Retract failed (non-fatal) — proceeding to HOME")

        # ── HOME: joint-space (gripper empty → wrist reorientation safe) ──────
        self._arm.max_velocity = self._arm_default_velocity
        self._arm.max_acceleration = self._arm_default_acceleration
        self.get_logger().info("[HOME] Returning to home (object released)")
        t0 = time.time()
        self._arm.move_to_configuration(
            joint_positions=HOME_JOINT_POSITIONS, joint_names=robot.joint_names()
        )
        ok = self._arm.wait_until_executed()
        self.get_logger().info(
            f"[HOME] {'Reached' if ok else 'FAILED'} in {time.time()-t0:.2f}s"
        )
        return True  # place succeeded


def main(args=None):
    rclpy.init(args=args)
    node = GraspExecutorNode()
    executor = MultiThreadedExecutor(node.get_parameter('executor_threads').get_parameter_value().integer_value)
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
