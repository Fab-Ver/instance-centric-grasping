"""RViz digital twin of the Gazebo scene: live object meshes + open-top bin.

Each object gets its own moving TF frame (`<entity>_viz`) broadcast from the gz
pose stream, and its marker is `frame_locked` in that frame. RViz then retransforms
the marker to the latest TF every render cycle — the same mechanism the RobotModel
display uses — so the mesh stays perfectly synced to the gripper with zero relative
lag. The marker definitions are static (only the visual-local offset); all motion
travels through TF.
"""
import os
import threading

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from icgnet_msgs.msg import SceneManifest
from icgnet_main.scene_utils import find_model_sdf, visual_geometry_from_sdf


# Bin geometry from worlds/icgnet_world.sdf drop_bin (static, pose 0.45 -0.50 0, orange).
_BIN_X, _BIN_Y, _BIN_Z = 0.45, -0.50, 0.0
_BIN_R, _BIN_G, _BIN_B, _BIN_A = 0.9, 0.5, 0.0, 0.7
# (link_offset_xyz, cube_scale_xyz) for floor and four walls.
_BIN_PARTS = [
    ((0.0,    0.0,    0.0025), (0.30, 0.30, 0.005)),   # floor
    ((0.14,   0.0,    0.05),   (0.02, 0.30, 0.10)),    # wall +X
    ((-0.14,  0.0,    0.05),   (0.02, 0.30, 0.10)),    # wall -X
    ((0.0,    0.14,   0.05),   (0.26, 0.02, 0.10)),    # wall +Y
    ((0.0,   -0.14,   0.05),   (0.26, 0.02, 0.10)),    # wall -Y
]
_BIN_ID_BASE = 900  # marker ids 900-904 (above all entity ids)

# Marker-definition republish rate. Motion is carried by TF + frame_locked, NOT by
# this timer; it only re-asserts static marker defs + bin and handles DELETE.
_REPUBLISH_HZ = 10.0


class SceneVisualizerNode(Node):
    def __init__(self):
        super().__init__('scene_visualizer')

        pkg_share = get_package_share_directory('icgnet_main')
        self._models_dir = os.path.join(pkg_share, 'models')

        self._lock = threading.Lock()
        # entity_name → (translation np[3], quaternion np[4] xyzw) — latest gz pose.
        self._entity_poses: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        # entity_name → model_name (populated from manifests)
        self._registry: dict[str, str] = {}
        # entity_name → stable marker id
        self._entity_id_map: dict[str, int] = {}
        self._next_entity_id: int = 0

        self._geom_lock = threading.Lock()
        # model_name → parsed spec or None
        self._geom_cache: dict[str, dict | None] = {}

        # Entity names rendered in the previous tick (for targeted DELETE on removal).
        self._prev_entities: set[str] = set()

        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        poses_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._tf_broadcaster = TransformBroadcaster(self)

        cb = ReentrantCallbackGroup()
        self.create_subscription(
            TFMessage, '/model_poses', self._model_poses_cb, poses_qos, callback_group=cb,
        )
        # multi-object (scene_manager) and single/replay manifest on separate topic.
        self.create_subscription(
            SceneManifest, '/icgnet/scene_manifest',
            self._manifest_cb, latched_qos, callback_group=cb,
        )
        self.create_subscription(
            SceneManifest, '/icgnet/scene_manifest_viz',
            self._manifest_cb, latched_qos, callback_group=cb,
        )
        self._pub = self.create_publisher(MarkerArray, '/icgnet/scene_meshes', 2)
        self.create_timer(
            1.0 / _REPUBLISH_HZ, self._publish_markers, callback_group=cb,
        )
        self.get_logger().info(
            'SceneVisualizerNode ready — per-object TF frames + frame_locked markers '
            f'on /icgnet/scene_meshes (defs republished at {_REPUBLISH_HZ:.0f} Hz).'
        )

    # ── subscription callbacks ─────────────────────────────────────────────────

    def _model_poses_cb(self, msg: TFMessage):
        now = self.get_clock().now().to_msg()
        tf_out: list[TransformStamped] = []
        with self._lock:
            for tf in msg.transforms:
                entity = tf.child_frame_id
                if entity not in self._registry:
                    continue  # ignore robot links and other unknown frames
                t = tf.transform.translation
                q = tf.transform.rotation
                self._entity_poses[entity] = (
                    np.array([t.x, t.y, t.z], dtype=float),
                    np.array([q.x, q.y, q.z, q.w], dtype=float),
                )
                out = TransformStamped()
                stamp = tf.header.stamp
                out.header.stamp = stamp if (stamp.sec or stamp.nanosec) else now
                out.header.frame_id = 'world'
                out.child_frame_id = f'{entity}_viz'
                out.transform.translation = t
                out.transform.rotation = q
                tf_out.append(out)

        if tf_out:
            # Event-driven, same immediacy as the robot's /tf — zero added latency.
            self._tf_broadcaster.sendTransform(tf_out)

    def _manifest_cb(self, msg: SceneManifest):
        with self._lock:
            for obj in msg.objects:
                self._registry[obj.entity_name] = obj.model_name
                if obj.entity_name not in self._entity_id_map:
                    self._entity_id_map[obj.entity_name] = self._next_entity_id
                    self._next_entity_id += 1
        self.get_logger().info(
            '[VIZ] Manifest: '
            + ', '.join(f'{o.entity_name}({o.model_name})' for o in msg.objects)
        )

    # ── geometry spec (lazy, cached) ───────────────────────────────────────────

    def _get_geom_spec(self, model_name: str) -> dict | None:
        with self._geom_lock:
            if model_name in self._geom_cache:
                return self._geom_cache[model_name]
        # SDF parsing happens outside the lock (file I/O, does not block callbacks).
        sdf_path = find_model_sdf(self._models_dir, model_name)
        if sdf_path is None:
            self.get_logger().warn(f'[VIZ] SDF not found for model: {model_name!r}')
            spec = None
        else:
            spec = visual_geometry_from_sdf(sdf_path)
            if spec is None:
                self.get_logger().warn(f'[VIZ] Cannot parse visual geometry: {sdf_path}')
        with self._geom_lock:
            self._geom_cache[model_name] = spec
        return spec

    # ── marker builders ────────────────────────────────────────────────────────

    def _build_entity_marker(
        self,
        entity_name: str,
        model_name: str,
        marker_id: int,
        stamp,
    ) -> Marker | None:
        spec = self._get_geom_spec(model_name)
        if spec is None:
            return None

        m = Marker()
        # Frame-locked in the object's own moving TF frame: RViz retransforms it to
        # the latest TF every render cycle (same as RobotModel) → zero relative lag.
        m.header.frame_id = f'{entity_name}_viz'
        m.header.stamp = stamp
        m.frame_locked = True
        m.ns = 'scene_meshes'
        m.id = marker_id
        m.action = Marker.ADD
        m.lifetime.sec = 0
        m.lifetime.nanosec = 0

        # Pose carries only the visual-local offset; the world pose is in the TF.
        p_off = spec['offset_xyz']
        q_off = Rotation.from_euler('xyz', spec['offset_rpy']).as_quat()
        m.pose.position.x = float(p_off[0])
        m.pose.position.y = float(p_off[1])
        m.pose.position.z = float(p_off[2])
        m.pose.orientation.x = float(q_off[0])
        m.pose.orientation.y = float(q_off[1])
        m.pose.orientation.z = float(q_off[2])
        m.pose.orientation.w = float(q_off[3])

        r, g, b, a = spec['color_rgba']
        m.color.r = float(r)
        m.color.g = float(g)
        m.color.b = float(b)
        m.color.a = float(a)

        kind = spec['kind']
        if kind == 'mesh':
            m.type = Marker.MESH_RESOURCE
            m.mesh_resource = spec['uri']
            m.mesh_use_embedded_materials = True
            # color.a must be non-zero for embedded DAE materials to render.
            m.color.a = 1.0
            sx, sy, sz = spec['scale']
            m.scale.x = float(sx)
            m.scale.y = float(sy)
            m.scale.z = float(sz)
        elif kind == 'cylinder':
            m.type = Marker.CYLINDER
            d = 2.0 * spec['radius']   # RViz scale = full diameter
            m.scale.x = float(d)
            m.scale.y = float(d)
            m.scale.z = float(spec['length'])
        elif kind == 'box':
            m.type = Marker.CUBE
            dx, dy, dz = spec['dims']
            m.scale.x = float(dx)
            m.scale.y = float(dy)
            m.scale.z = float(dz)
        elif kind == 'sphere':
            m.type = Marker.SPHERE
            d = 2.0 * spec['radius']   # RViz scale = full diameter
            m.scale.x = float(d)
            m.scale.y = float(d)
            m.scale.z = float(d)
        else:
            return None

        return m

    def _build_bin_markers(self, stamp) -> list[Marker]:
        markers = []
        for i, (offset, scale) in enumerate(_BIN_PARTS):
            m = Marker()
            m.header.frame_id = 'world'
            m.header.stamp = stamp
            m.ns = 'scene_meshes'
            m.id = _BIN_ID_BASE + i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.lifetime.sec = 0
            m.lifetime.nanosec = 0
            m.pose.position.x = _BIN_X + offset[0]
            m.pose.position.y = _BIN_Y + offset[1]
            m.pose.position.z = _BIN_Z + offset[2]
            m.pose.orientation.w = 1.0
            m.scale.x = float(scale[0])
            m.scale.y = float(scale[1])
            m.scale.z = float(scale[2])
            m.color.r = _BIN_R
            m.color.g = _BIN_G
            m.color.b = _BIN_B
            m.color.a = _BIN_A
            markers.append(m)
        return markers

    # ── timer: republish static marker definitions + bin + DELETE handling ──────

    def _publish_markers(self):
        stamp = self.get_clock().now().to_msg()

        with self._lock:
            registry = dict(self._registry)
            live_entities = set(self._entity_poses.keys())
            id_map = dict(self._entity_id_map)

        markers: list[Marker] = []
        current_entities: set[str] = set()

        for entity_name, model_name in registry.items():
            if entity_name not in live_entities:
                continue  # no pose received yet → no TF frame → skip
            marker_id = id_map.get(entity_name)
            if marker_id is None:
                continue
            m = self._build_entity_marker(entity_name, model_name, marker_id, stamp)
            if m is not None:
                markers.append(m)
                current_entities.add(entity_name)

        # Targeted DELETE for entities that disappeared since the last tick.
        for entity_name in self._prev_entities - current_entities:
            mid = id_map.get(entity_name)
            if mid is not None:
                dm = Marker()
                dm.header.frame_id = 'world'
                dm.header.stamp = stamp
                dm.ns = 'scene_meshes'
                dm.id = mid
                dm.action = Marker.DELETE
                markers.append(dm)

        self._prev_entities = current_entities
        markers.extend(self._build_bin_markers(stamp))

        if markers:
            ma = MarkerArray()
            ma.markers = markers
            self._pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = SceneVisualizerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
