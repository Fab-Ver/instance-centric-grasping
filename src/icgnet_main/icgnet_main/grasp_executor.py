import time
from threading import Lock, Thread

import numpy as np
import rclpy
import rclpy.duration
from geometry_msgs.msg import Point, Pose, PoseStamped
from tf2_msgs.msg import TFMessage
from shape_msgs.msg import SolidPrimitive
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from moveit_msgs.msg import (
    CollisionObject, PlanningScene, PlanningSceneComponents,
)
from moveit_msgs.srv import GetPlanningScene
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from pymoveit2 import MoveIt2, MoveIt2Gripper
from pymoveit2.robots import panda as robot

from icgnet_msgs.msg import GraspArray, SceneManifest
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
        self.declare_parameter('default_max_attempts', 5)
        self.declare_parameter('approach_offset', 0.12)
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
        self.declare_parameter('place_y', -0.50)
        self.declare_parameter('place_release_z', 0.26)
        self.declare_parameter('place_pre_offset', 0.15)
        self.declare_parameter('bin_footprint', 0.30)
        self.declare_parameter('bin_rim_height', 0.10)
        # z-margin above the rim for the in-bin check. Generous: objects can pile/tilt
        # on top of each other in a small bin (two cans stacked ≈ 0.14m centre). Only
        # guards against the object still hanging from the gripper/arm (z ≈ 0.30+).
        self.declare_parameter('bin_z_tolerance', 0.20)
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
        self.declare_parameter('pre_pos_z_min', 0.10)
        self.declare_parameter('arm_default_velocity', 0.5)
        self.declare_parameter('arm_default_acceleration', 0.5)
        self.declare_parameter('allowed_planning_time', 5.0)
        self.declare_parameter('num_planning_attempts', 10)
        self.declare_parameter('inference_timeout', 120.0)
        self.declare_parameter('bin_slot_spacing', 0.12)
        self.declare_parameter('reset_scene_service', '/icgnet/reset_scene')

        self._use_collision_scene = self.get_parameter('use_collision_scene').get_parameter_value().bool_value
        self._attach_weight = self.get_parameter('attach_weight').get_parameter_value().double_value
        self._place_x = self.get_parameter('place_x').get_parameter_value().double_value
        self._place_y = self.get_parameter('place_y').get_parameter_value().double_value
        self._place_release_z = self.get_parameter('place_release_z').get_parameter_value().double_value
        self._place_pre_offset = self.get_parameter('place_pre_offset').get_parameter_value().double_value
        self._bin_footprint = self.get_parameter('bin_footprint').get_parameter_value().double_value
        self._bin_rim_height = self.get_parameter('bin_rim_height').get_parameter_value().double_value
        self._bin_z_tolerance = self.get_parameter('bin_z_tolerance').get_parameter_value().double_value
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
        self._pre_pos_z_min = self.get_parameter('pre_pos_z_min').get_parameter_value().double_value
        self._arm_default_velocity = self.get_parameter('arm_default_velocity').get_parameter_value().double_value
        self._arm_default_acceleration = self.get_parameter('arm_default_acceleration').get_parameter_value().double_value
        self._allowed_planning_time = self.get_parameter('allowed_planning_time').get_parameter_value().double_value
        self._num_planning_attempts = self.get_parameter('num_planning_attempts').get_parameter_value().integer_value
        self._inference_timeout = self.get_parameter('inference_timeout').get_parameter_value().double_value
        self._bin_slot_spacing = self.get_parameter('bin_slot_spacing').get_parameter_value().double_value
        self._reset_scene_svc_name = self.get_parameter('reset_scene_service').get_parameter_value().string_value

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
        self._object_pose_pub = self.create_publisher(PoseStamped, '/icgnet/object_pose', 1)

        self._set_entity_client = self.create_client(
            SetEntityPose, '/world/icgnet_world/set_pose', callback_group=cb_svc
        )

        self._get_scene_client = self.create_client(
            GetPlanningScene, '/get_planning_scene', callback_group=cb_svc
        )
        self._reset_scene_client = self.create_client(
            Trigger, self._reset_scene_svc_name, callback_group=cb_svc
        )

        self._co_pub = self.create_publisher(CollisionObject, '/collision_object', 10)

        # Tracks the CO that is currently being managed so _reset_scene() can
        # remove it before planning (arm may be inside the CO after a failed attempt).
        self._active_co_id: str | None = None
        self._active_co: CollisionObject | None = None

        # Object pose from gz-sim dynamic_pose/info — used for physical success check.
        self._object_pose_lock = Lock()
        self._object_pose: Point | None = None
        # Authoritative teleport-back target: the object's real pose captured from
        # /model_poses at inference time (before any arm motion). Used by _reset_scene
        # instead of the spawn-pose topic / yaml fallback, which can be stale/wrong.
        self._reset_target_pose: Point | None = None
        self.create_subscription(
            TFMessage, '/model_poses', self._model_poses_cb, 10, callback_group=cb_sub
        )

        self.create_service(ExecuteGrasp, '/icgnet/execute_grasp', self._execute_grasp_cb, callback_group=cb_grasp_exec)

        # Published lazily on the first execute_grasp call, when move_group is ready.
        self._bin_co_published = False

        # Evaluation instrumentation: set True when a Cartesian approach move fails
        # (collision / INVALID_MOTION_PLAN proxy).  Reset before each attempt loop.
        self._last_collision_detected = False

        # Multi-object sweep state.  Set when a SceneManifest arrives from scene_manager.
        # While False, every execute_grasp call uses the single-object path (unchanged).
        self._multi_object = False
        self._manifest: SceneManifest | None = None
        self._manifest_lock = Lock()

        # Per-entity pose dict populated from /model_poses for all dynamic entities.
        # Used by the sweep ground-truth check; the existing _object_pose scalar
        # (single-object path) is updated in parallel.
        self._entity_poses: dict[str, Point] = {}

        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            SceneManifest,
            '/icgnet/scene_manifest',
            self._manifest_cb,
            latched_qos,
            callback_group=cb_sub,
        )

        self.get_logger().info('GraspExecutorNode ready.')

    def _grasps_cb(self, msg: GraspArray):
        with self._grasps_lock:
            self._latest_grasps = msg

    def _manifest_cb(self, msg: SceneManifest):
        with self._manifest_lock:
            self._manifest = msg
            self._multi_object = True
        n_targets = sum(1 for o in msg.objects if o.entity_name.startswith('target_obj'))
        n_dist = sum(1 for o in msg.objects if o.entity_name.startswith('distractor'))
        self.get_logger().info(
            f'[MANIFEST] Scene manifest received: {len(msg.objects)} objects '
            f'({n_targets} targets, {n_dist} distractors) — multi-object sweep enabled'
        )

    def _model_poses_cb(self, msg: TFMessage):
        _tracked_pt = None
        _tracked_header = None
        with self._object_pose_lock:
            for transform in msg.transforms:
                t = transform.transform.translation
                pt = Point(x=t.x, y=t.y, z=t.z)
                self._entity_poses[transform.child_frame_id] = pt
                if transform.child_frame_id == self._object_entity_name:
                    self._object_pose = pt
                    _tracked_pt = pt
                    _tracked_header = transform.header
        if _tracked_pt is not None:
            ps = PoseStamped()
            ps.header.frame_id = 'world'
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x = _tracked_pt.x
            ps.pose.position.y = _tracked_pt.y
            ps.pose.position.z = _tracked_pt.z
            ps.pose.orientation.w = 1.0
            self._object_pose_pub.publish(ps)

    def _object_in_bin(self) -> bool | None:
        """Return True if the tracked object is inside the drop bin footprint.

        Returns None when no pose is available (bridge not yet connected).
        Uses bin_z_tolerance above the rim to tolerate piled/tilted objects.
        """
        with self._object_pose_lock:
            pose = self._object_pose
        if pose is None:
            return None
        return self._pose_in_bin(pose)

    def _pose_in_bin(self, pose: Point) -> bool:
        """Return True if pose is within the bin footprint (generalised _object_in_bin)."""
        half = self._bin_footprint / 2.0
        in_x = abs(pose.x - self._place_x) <= half
        in_y = abs(pose.y - self._place_y) <= half
        in_z = pose.z <= self._bin_rim_height + self._bin_z_tolerance
        return in_x and in_y and in_z

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
            # Reset target: the object's real pose captured at inference time
            # (from /model_poses), with the yaml pose as last-resort fallback.
            if self._reset_target_pose is not None:
                rx, ry, rz = (self._reset_target_pose.x,
                              self._reset_target_pose.y,
                              self._reset_target_pose.z)
            else:
                self.get_logger().warn('[RESET] No object pose from /model_poses — using yaml fallback.')
                rx, ry, rz = self._object_init_x, self._object_init_y, self._object_init_z

            # Skip the teleport if the object is essentially still where it should be:
            # a planning/pre-grasp failure never touched it, and teleporting would
            # reset its orientation (corrupting non-symmetric objects like the box)
            # and shift it away from the grasp poses ICGNet predicted.
            with self._object_pose_lock:
                cur = self._object_pose
            if cur is not None and np.hypot(cur.x - rx, cur.y - ry) < 0.03:
                self.get_logger().info(
                    '[RESET] Object undisturbed (<3cm from target) — skipping teleport.'
                )
            elif self._set_entity_client.wait_for_service(timeout_sec=2.0):
                req = SetEntityPose.Request()
                req.entity.name = self._object_entity_name
                req.entity.type = Entity.MODEL
                req.pose.position.x = rx
                req.pose.position.y = ry
                req.pose.position.z = rz
                req.pose.orientation.w = 1.0
                fut = self._set_entity_client.call_async(req)
                if not self._wait_for_future(fut):
                    self.get_logger().warn('[RESET] Object reset timed out.')
                elif fut.result().success:
                    self.get_logger().info(
                        f'[RESET] Object "{self._object_entity_name}" reset to [{rx:.3f}, {ry:.3f}, {rz:.3f}]'
                    )
                else:
                    self.get_logger().warn('[RESET] Object reset service returned failure.')
            else:
                self.get_logger().warn('[RESET] /world/icgnet_world/set_pose not available — object not reset.')

        # Re-add CO so the next attempt's Step-1 planning can avoid the object.
        if self._use_collision_scene and self._active_co is not None:
            self._readd_co_to_scene(self._active_co)
            self.get_logger().info(f"[RESET] Re-added CO '{self._active_co_id}' to scene.")

    def _full_reset(self):
        """Return arm home + restore all scene objects via /icgnet/reset_scene."""
        # Remove active CO and move arm home before calling the external reset service.
        if self._use_collision_scene and self._active_co_id is not None:
            if self._active_co is None:
                self._active_co = self._fetch_co_from_scene(self._active_co_id)
            self._remove_co_from_scene(self._active_co_id)
            time.sleep(0.15)

        self.get_logger().info('[FULL RESET] Opening gripper...')
        self._gripper.open()
        self._gripper.wait_until_executed()

        self.get_logger().info('[FULL RESET] Moving arm to home...')
        self._arm.max_velocity = self._arm_default_velocity
        self._arm.max_acceleration = self._arm_default_acceleration
        self._arm.move_to_configuration(
            joint_positions=HOME_JOINT_POSITIONS, joint_names=robot.joint_names()
        )
        ok = self._arm.wait_until_executed()
        self.get_logger().info(f'[FULL RESET] Arm at home: {ok}')

        self._active_co_id = None
        self._active_co = None

        if self._reset_scene_client.wait_for_service(timeout_sec=3.0):
            fut = self._reset_scene_client.call_async(Trigger.Request())
            if self._wait_for_future(fut, timeout=30.0) and fut.result() is not None:
                self.get_logger().info(
                    f'[FULL RESET] Scene reset: {fut.result().message}'
                )
            else:
                self.get_logger().warn('[FULL RESET] Scene reset timed out or failed.')
        else:
            self.get_logger().warn(
                f'[FULL RESET] {self._reset_scene_svc_name} not available — objects not reset.'
            )

    def _partial_reset_for_instance(self, grasp, manifest: SceneManifest) -> None:
        """Arm home + gripper open + teleport ALL manifest entities not yet in the bin.

        Resets the entire scene (except objects already placed in the bin) so that
        subsequent retry attempts find objects at the ICGNet-predicted positions.
        Resetting only the active entity is insufficient: failed approach moves can
        physically displace distractors and other targets, causing all further attempts
        to miss even after the active entity is restored.
        """
        if self._use_collision_scene and self._active_co_id is not None:
            if self._active_co is None:
                self._active_co = self._fetch_co_from_scene(self._active_co_id)
            self._remove_co_from_scene(self._active_co_id)
            time.sleep(0.15)

        self.get_logger().info('[PARTIAL RESET] Opening gripper...')
        self._gripper.open()
        self._gripper.wait_until_executed()

        self.get_logger().info('[PARTIAL RESET] Moving arm to home...')
        self._arm.max_velocity = self._arm_default_velocity
        self._arm.max_acceleration = self._arm_default_acceleration
        self._arm.move_to_configuration(
            joint_positions=HOME_JOINT_POSITIONS, joint_names=robot.joint_names()
        )
        ok = self._arm.wait_until_executed()
        self.get_logger().info(f'[PARTIAL RESET] Arm at home: {ok}')

        self._active_co_id = None
        self._active_co = None

        if not self._set_entity_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('[PARTIAL RESET] set_pose not available — entities not reset.')
            return

        with self._object_pose_lock:
            poses_snapshot = dict(self._entity_poses)

        n_reset = n_skipped = 0
        for obj in manifest.objects:
            current = poses_snapshot.get(obj.entity_name)
            if current is not None and self._pose_in_bin(current):
                n_skipped += 1
                continue
            req = SetEntityPose.Request()
            req.entity.name = obj.entity_name
            req.entity.type = Entity.MODEL
            req.pose = obj.pose
            fut = self._set_entity_client.call_async(req)
            if not self._wait_for_future(fut):
                self.get_logger().warn(
                    f"[PARTIAL RESET] Teleport timed out for '{obj.entity_name}'"
                )
            elif fut.result() and not fut.result().success:
                self.get_logger().warn(
                    f"[PARTIAL RESET] Teleport failed for '{obj.entity_name}'"
                )
            else:
                n_reset += 1
        self.get_logger().info(
            f'[PARTIAL RESET] Teleported {n_reset} entities to spawn poses '
            f'({n_skipped} already in bin, skipped).'
        )

    def _execute_grasp_cb(self, req: ExecuteGrasp.Request, res: ExecuteGrasp.Response):
        with self._manifest_lock:
            multi = self._multi_object
        if multi:
            return self._execute_sweep(req, res)

        target = req.target.strip() if req.target else 'any'
        max_attempts = req.max_attempts if req.max_attempts > 0 else self._default_max_attempts
        skip_place = req.skip_place

        res.planning_time = 0.0
        res.execution_time = 0.0
        res.collision_detected = False

        self.get_logger().info(
            f"ExecuteGrasp: target='{target}' max_attempts={max_attempts} skip_place={skip_place}"
        )

        # Ensure the drop-bin collision object is in the planning scene.
        if not self._bin_co_published:
            self._add_bin_collision_object()
            self._bin_co_published = True

        # Clear stale grasps so we know the next GraspArray is fresh
        with self._grasps_lock:
            self._latest_grasps = None

        # Return arm to HOME and open gripper BEFORE triggering inference.
        # At inference time the camera must see only the object: if the arm is at
        # a post-grasp position (above object, mid-lift, etc.) it enters the camera
        # FOV and ICGNet reconstructs it as a spurious instance, producing false
        # collision objects and degrading grasp selection.
        self.get_logger().info('[RESET] Moving arm to home before inference (clean scene for ICGNet)...')
        self._reset_scene(teleport_object=False)

        if not self._compute_client.wait_for_service(timeout_sec=5.0):
            res.success = False
            res.message = f"Service '{self._compute_client.srv_name}' not available"
            return res

        t_plan_start = time.time()
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

        res.planning_time = time.time() - t_plan_start
        res.target_not_found = False

        if grasps is None or len(grasps.grasps) == 0:
            res.success = False
            res.target_not_found = True
            res.message = "No grasps received after inference"
            return res

        candidates = self._filter_grasps(grasps.grasps, target)
        if not candidates:
            res.target_not_found = True
            available = sorted({CLASS_NAMES.get(g.semantic_class, '?') for g in grasps.grasps})
            res.success = False
            res.message = (
                f"No grasps after filtering: target='{target}' — "
                f"ICGNet predicted classes: {available}"
            )
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

        # Capture the object's real pose now (arm at home, object untouched) so any
        # inter-attempt reset teleports it back exactly where ICGNet saw it.
        with self._object_pose_lock:
            self._reset_target_pose = self._object_pose
        if self._reset_target_pose is not None:
            self.get_logger().info(
                f"[RESET] Teleport-back target = "
                f"[{self._reset_target_pose.x:.3f}, {self._reset_target_pose.y:.3f}, "
                f"{self._reset_target_pose.z:.3f}] (from /model_poses)"
            )

        self._last_collision_detected = False
        t_exec_start = time.time()

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
                res.execution_time = time.time() - t_exec_start
                res.collision_detected = self._last_collision_detected
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
        res.execution_time = time.time() - t_exec_start
        res.collision_detected = self._last_collision_detected
        res.message = f"All {len(candidates)} grasp attempts failed"
        return res

    def _bin_grid_slots(self, n: int) -> list[tuple[float, float]]:
        """Return n evenly-spaced drop slots within the bin, centred along the Y axis."""
        if n <= 1:
            return [(self._place_x, self._place_y)]
        y_start = self._place_y - (n - 1) * self._bin_slot_spacing / 2.0
        return [(self._place_x, y_start + i * self._bin_slot_spacing) for i in range(n)]

    def _execute_sweep(
        self, req: ExecuteGrasp.Request, res: ExecuteGrasp.Response
    ) -> ExecuteGrasp.Response:
        """Multi-object sweep: collect every instance of the target class, then HOME.

        Triggered automatically when a SceneManifest has been received.  Uses a single
        ICGNet inference pass, groups grasps by instance_id, and attempts each instance
        in best-score order.  On any single-instance failure the scene is fully reset.
        On success, verifies all target entities from the manifest are inside the bin.
        """
        target = req.target.strip() if req.target else 'any'
        max_attempts = req.max_attempts if req.max_attempts > 0 else self._default_max_attempts
        skip_place = req.skip_place

        with self._manifest_lock:
            manifest = self._manifest

        if manifest is None:
            res.success = False
            res.message = "No scene manifest — cannot run multi-object sweep"
            return res

        self.get_logger().info(
            f"[SWEEP] target='{target}' max_attempts={max_attempts} skip_place={skip_place}"
        )

        if not self._bin_co_published:
            self._add_bin_collision_object()
            self._bin_co_published = True

        with self._grasps_lock:
            self._latest_grasps = None

        # Move arm home + open gripper so the full scene is visible to ICGNet.
        self._reset_scene(teleport_object=False)

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

        candidates = self._filter_grasps(grasps.grasps, target)
        if not candidates:
            available = sorted({CLASS_NAMES.get(g.semantic_class, '?') for g in grasps.grasps})
            if target != 'any':
                target_class_id = SEMANTIC_CLASSES.get(target)
                has_target_in_manifest = any(
                    o.semantic_class == target_class_id and o.entity_name.startswith('target_obj')
                    for o in manifest.objects
                ) if target_class_id is not None else False
                target_seen_in_grasps = (
                    target_class_id is not None
                    and any(g.semantic_class == target_class_id for g in grasps.grasps)
                )
                if not has_target_in_manifest:
                    msg = (
                        f"No '{target}' objects in scene manifest — "
                        f"spawn with target_class:={target} first"
                    )
                elif target_seen_in_grasps:
                    msg = (
                        f"All '{target}' grasps rejected by kinematic filters "
                        f"(pre_pos_z_min={self._pre_pos_z_min:.2f}m) — "
                        f"ICGNet predicted only horizontal grasps at low z. "
                        f"Check [FILTER] log for breakdown."
                    )
                else:
                    msg = (
                        f"ICGNet did not detect class '{target}' — "
                        f"predicted classes: {available}"
                    )
            else:
                msg = f"No grasps after filtering — ICGNet found no objects (predicted: {available})"
            res.success = False
            res.message = msg
            self.get_logger().error(f'[SWEEP] {msg}')
            return res

        # Group grasps by instance_id; order instances by best score (descending).
        from collections import defaultdict
        instance_grasps: dict[int, list] = defaultdict(list)
        for g in candidates:
            instance_grasps[g.instance_id].append(g)
        instances = sorted(
            instance_grasps.keys(),
            key=lambda iid: instance_grasps[iid][0].score,
            reverse=True,
        )
        n_instances = len(instances)
        slots = self._bin_grid_slots(n_instances)
        self.get_logger().info(
            f"[SWEEP] {n_instances} instance(s) to collect: {instances}"
        )

        # Fetch all ICGNet instance COs upfront.  Each attempt runs with only the active
        # instance CO in the planning scene — other COs block joint-space pre-grasp
        # planning for far-reaching poses (x > 0.85 m from base).
        inst_cos: dict[int, CollisionObject | None] = {}
        if self._use_collision_scene:
            req = GetPlanningScene.Request()
            req.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            fut = self._get_scene_client.call_async(req)
            if self._wait_for_future(fut):
                for co in fut.result().scene.world.collision_objects:
                    if co.id.startswith(self._collision_id_prefix):
                        try:
                            iid = int(co.id[len(self._collision_id_prefix):])
                            inst_cos[iid] = co
                        except ValueError:
                            pass
            for co in inst_cos.values():
                if co is not None:
                    self._remove_co_from_scene(co.id)
            if inst_cos:
                time.sleep(0.15)

        total_attempted = 0
        for slot_idx, inst_id in enumerate(instances):
            inst_candidates = instance_grasps[inst_id][:max_attempts]
            place_xy = slots[slot_idx]
            self.get_logger().info(
                f"[SWEEP] Instance {inst_id}: {len(inst_candidates)} candidate(s), "
                f"slot=({place_xy[0]:.3f}, {place_xy[1]:.3f})"
            )

            success = False
            for attempt_idx, g in enumerate(inst_candidates):
                if attempt_idx > 0:
                    # Partial reset: a failed attempt may have knocked the object away from
                    # the inference position.  Teleport only this instance's entity back to
                    # its spawn pose — objects already placed in the bin are not disturbed.
                    self._partial_reset_for_instance(g, manifest)

                # Restore only the active instance CO so pre-grasp planning is unobstructed.
                if self._use_collision_scene and inst_cos:
                    for iid, co in inst_cos.items():
                        if iid != inst_id and co is not None:
                            self._remove_co_from_scene(co.id)
                    active_co = inst_cos.get(inst_id)
                    if active_co is not None:
                        self._readd_co_to_scene(active_co)

                total_attempted += 1
                self.get_logger().info(
                    f"[SWEEP] inst={inst_id} attempt {attempt_idx + 1}/{len(inst_candidates)} "
                    f"score={g.score:.3f}"
                )
                ok = self._execute_single_grasp(g, skip_place=skip_place, place_xy=place_xy)
                if ok:
                    success = True
                    break
                self.get_logger().warn(
                    f"[SWEEP] inst={inst_id} attempt {attempt_idx + 1} failed"
                )

            if not success:
                self.get_logger().error(
                    f"[SWEEP] Failed to grasp instance {inst_id} — triggering full reset"
                )
                self._full_reset()
                self._clear_current_grasp_marker()
                res.success = False
                res.grasps_attempted = total_attempted
                res.message = (
                    f"Sweep failed: could not grasp instance {inst_id} after "
                    f"{len(inst_candidates)} attempt(s) — scene reset"
                )
                return res

        # Ground-truth check: every target entity from the manifest must be in the bin.
        if not skip_place:
            time.sleep(1.0)  # let objects settle after last release
            target_class_id = SEMANTIC_CLASSES.get(target)
            target_entity_names = [
                o.entity_name for o in manifest.objects
                if o.entity_name.startswith('target_obj') and (
                    target == 'any' or o.semantic_class == target_class_id
                )
            ]
            with self._object_pose_lock:
                poses_snapshot = dict(self._entity_poses)
            missing = [
                name for name in target_entity_names
                if name not in poses_snapshot or not self._pose_in_bin(poses_snapshot[name])
            ]
            # [BIN_DIAG] log the actual tracked pose of every target vs bin bounds —
            # distinguishes "stale/high pose" (gz stops publishing static entities on
            # dynamic_pose/info) from a genuine z/footprint miss.
            half = self._bin_footprint / 2.0
            z_max = self._bin_rim_height + self._bin_z_tolerance
            for name in target_entity_names:
                p = poses_snapshot.get(name)
                if p is None:
                    self.get_logger().warn(f"[BIN_DIAG] {name}: NO POSE in snapshot (entity not tracked)")
                else:
                    self.get_logger().info(
                        f"[BIN_DIAG] {name}: pos=({p.x:.3f},{p.y:.3f},{p.z:.3f}) "
                        f"in_bin={self._pose_in_bin(p)} "
                        f"[x|y|±{half:.2f} from ({self._place_x:.2f},{self._place_y:.2f}); z≤{z_max:.2f}]"
                    )
            if missing:
                self.get_logger().error(
                    f"[SWEEP] Ground-truth check failed — not in bin: {missing}. Full reset."
                )
                self._full_reset()
                self._clear_current_grasp_marker()
                res.success = False
                res.grasps_attempted = total_attempted
                res.message = (
                    f"Sweep failed: {missing} not confirmed in bin after collection — scene reset"
                )
                return res

        self._clear_current_grasp_marker()
        res.success = True
        res.grasps_attempted = total_attempted
        res.message = (
            f"Sweep complete: collected {n_instances} instance(s) of class '{target}'"
        )
        self.get_logger().info(f"[SWEEP] SUCCESS — {res.message}")
        return res

    def _filter_grasps(self, grasps, target: str) -> list:
        filtered = []
        n_total = len(grasps)
        n_ws = n_width = n_target = n_low = 0
        for g in grasps:
            p = g.pose.position
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
            # Reject grasps where the pre-approach position would be too low for the arm
            # to reach from home.  For horizontal approaches (approach_z≈0), pre_pos.z =
            # grasp.z; near-horizontal grasps at low z hit joint limits and RRTConnect
            # exhausts the planning timeout.  Top-down grasps add approach_offset along
            # z so they always clear this threshold.
            q = g.pose.orientation
            approach_z = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()[2, 2]
            pre_pos_z = p.z - self._approach_offset * approach_z
            if pre_pos_z < self._pre_pos_z_min:
                n_low += 1
                continue
            filtered.append(g)

        filtered.sort(key=lambda g: g.score, reverse=True)
        score_range = (
            f"[{filtered[-1].score:.2f}–{filtered[0].score:.2f}]"
            if filtered else "—"
        )
        self.get_logger().info(
            f"[FILTER] total={n_total} → kept={len(filtered)} (scores={score_range}) | "
            f"rejected: width={n_width} workspace={n_ws} target={n_target} low_prepos={n_low}"
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

    def _execute_single_grasp(
        self, g, skip_place: bool = False, place_xy: tuple | None = None
    ) -> bool:
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
            self._last_collision_detected = True
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
        return self._place_object(quat_xyzw, target_co_id, skip_place, place_xy=place_xy)


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

    def _place_object(
        self,
        quat_xyzw: list,
        target_co_id: str,
        skip_place: bool,
        place_xy: tuple | None = None,
    ) -> bool:
        """Steps 5–7 + retract + HOME: transport → lower → release → retract → HOME.

        Called after Step 4 (Cartesian lift) with the object held. Three-phase deposit:
          Step 5/7: Joint-space transfer → (place_x, place_y, transport_clear_z).
                    OMPL plans freely over the long lateral arc; orientation is an IK goal on
                    the target, not a path constraint → pick_ik finds a valid seed without
                    hitting joint limits. Runs at transport_velocity/acceleration.
          Step 6/7: Cartesian lower → (place_x, place_y, place_release_z).
                    Short straight descent above the bin, orientation locked.
          Step 7/7: Open gripper + settle release_settle_states fresh joint_states (sim-time
                    settle, RTF-independent) → object falls free into bin.
          RETRACT:  Cartesian +Z to place_release_z + safe_retract_height (gripper empty).
          HOME:     Joint-space to HOME_JOINT_POSITIONS at arm_default_velocity.

        place_xy overrides place_x/place_y for multi-object sweep (per-instance bin slot).
        Physical success verified via _object_in_bin() (object pose from /model_poses).
        """
        place_x = place_xy[0] if place_xy is not None else self._place_x
        place_y = place_xy[1] if place_xy is not None else self._place_y

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

        transfer_pos = [place_x, place_y, self._transport_clear_z]
        place_release_pos = [place_x, place_y, self._place_release_z]
        retract_pos = [place_x, place_y, self._place_release_z + self._safe_retract_height]

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
        # Threshold 0.3: top-down grasps achieve fraction≈1.0 (full descent);
        # horizontal grasps hit an IK discontinuity at ≈50% and stop mid-way — still
        # accepted (0.5 ≥ 0.3) so the object is released ~5 cm higher and falls into bin.
        # Joint-space alternative causes 30s+ spinning via distant IK branches.
        self.get_logger().info(
            f"[STEP 6/7] LOWER → {[f'{v:.3f}' for v in place_release_pos]}"
        )
        t0 = time.time()
        self._arm.move_to_pose(
            position=place_release_pos, quat_xyzw=quat_xyzw,
            cartesian=True,
            cartesian_max_step=self._cartesian_max_step_approach,
            cartesian_fraction_threshold=0.3,
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

        # Physical success check: verify the object landed in the bin.
        result = self._object_in_bin()
        if result is None:
            self.get_logger().warn(
                "[PLACE] No object pose from /model_poses — physical check skipped "
                "(bridge not connected). Reporting success by arm motion only."
            )
            return True
        if result:
            with self._object_pose_lock:
                p = self._object_pose
            self.get_logger().info(
                f"[PLACE] SUCCESS: object in bin "
                f"(pose=[{p.x:.3f}, {p.y:.3f}, {p.z:.3f}], "
                f"bin=[{place_x:.3f}, {place_y:.3f}] ±{self._bin_footprint/2:.3f})"
            )
            return True
        with self._object_pose_lock:
            p = self._object_pose
        self.get_logger().warn(
            f"[PLACE] FAILURE: object NOT in bin after release — "
            f"pose=[{p.x:.3f}, {p.y:.3f}, {p.z:.3f}], "
            f"bin=[{place_x:.3f}, {place_y:.3f}] ±{self._bin_footprint/2:.3f}"
        )
        return False


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
