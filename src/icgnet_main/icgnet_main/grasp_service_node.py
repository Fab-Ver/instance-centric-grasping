import colorsys
import os

import numpy as np
import rclpy
import rclpy.duration
import rclpy.time
import tf2_ros
from geometry_msgs.msg import Point, Pose, PoseArray
from moveit_msgs.msg import CollisionObject
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2, PointField
from shape_msgs.msg import Mesh, MeshTriangle
from std_msgs.msg import ColorRGBA, Header
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from icgnet_msgs.msg import Grasp, GraspArray

from .icgnet_inference import ICGNetPredictor
from .pointcloud_utils import (
    PointCloudConfig, gripper_keypoints_world, pointcloud2_to_numpy, process_point_cloud_dual,
)


SCORE_HUE_GREEN = 0.33
MIN_POINTS_FOR_INFERENCE = 50
CLASS_NAMES = {0: 'mug', 1: 'box', 2: 'can', 3: 'bottle', 4: 'cylindric', 5: 'ball', 6: 'other'}


def _score_to_color(score: float) -> ColorRGBA:
    h = max(0.0, min(1.0, score)) * SCORE_HUE_GREEN
    r, g, b = colorsys.hsv_to_rgb(h, 1.0, 1.0)
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=0.9)



def _build_grasp_markers(centers, rot_matrices, scores, widths, frame_id, now):
    """
    Build a MarkerArray with gripper wireframe shapes (LINE_LIST) for each grasp.
    Each gripper = 4 line segments (2 fingers + crossbar + handle).
    Color encodes score: red=low, green=high.
    """
    ma = MarkerArray()

    clear = Marker()
    clear.header.frame_id = frame_id
    clear.header.stamp = now
    clear.action = Marker.DELETEALL
    ma.markers.append(clear)

    if len(centers) == 0:
        return ma

    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = now
    m.ns = "icgnet_grasps"
    m.id = 0
    m.type = Marker.LINE_LIST
    m.action = Marker.ADD
    m.scale.x = 0.004   # line width 4mm
    m.lifetime = rclpy.duration.Duration(seconds=60).to_msg()

    for c, R, s, w in zip(centers, rot_matrices, scores, widths):
        color = _score_to_color(float(s))
        w_clipped = float(np.clip(w, 0.02, 0.08))
        segments = gripper_keypoints_world(c, R, w_clipped)
        for start, end in segments:
            m.points.append(Point(x=float(start[0]), y=float(start[1]), z=float(start[2])))
            m.points.append(Point(x=float(end[0]),   y=float(end[1]),   z=float(end[2])))
            m.colors.append(color)
            m.colors.append(color)

    ma.markers.append(m)
    return ma


def _numpy_to_pointcloud2(points: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    """Convert (N, 3) float32 numpy array to a PointCloud2 message."""
    header = Header()
    header.frame_id = frame_id
    header.stamp = stamp
    fields = [
        PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
    ]
    point_step = 12
    data = points.astype(np.float32).tobytes()
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(points)
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = point_step
    msg.row_step = point_step * len(points)
    msg.data = data
    msg.is_dense = True
    return msg


class ICGNetGraspNode(Node):
    def __init__(self):
        super().__init__('icgnet_grasp_node')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('config_path', '')
        self.declare_parameter('icgnet_repo_path', '')
        self.declare_parameter('camera_topic', '/camera/rgbd_camera/points')
        self.declare_parameter('target_frame', 'world')
        self.declare_parameter('voxel_size', 0.003)
        self.declare_parameter('n_grasps', 32)
        self.declare_parameter('workspace_x_min', 0.25)
        self.declare_parameter('workspace_x_max', 1.05)
        self.declare_parameter('workspace_y_min', -0.50)
        self.declare_parameter('workspace_y_max', 0.50)
        self.declare_parameter('workspace_z_min', 0.01)
        self.declare_parameter('workspace_z_max', 0.60)
        self.declare_parameter('publish_collision_objects', True)
        self.declare_parameter('collision_use_convex_hull', True)
        self.declare_parameter('collision_object_topic', '/collision_object')
        self.declare_parameter('collision_id_prefix', 'icgnet_inst_')
        self.declare_parameter('return_meshes', True)
        self.declare_parameter('pc_nb_neighbors', 20)
        self.declare_parameter('pc_std_ratio', 2.0)
        self.declare_parameter('pc_normal_radius_factor', 5.0)
        self.declare_parameter('pc_normal_max_nn', 30)
        self.declare_parameter('exclude_bin', True)
        self.declare_parameter('exclude_bin_x', 0.45)
        self.declare_parameter('exclude_bin_y', -0.50)
        self.declare_parameter('exclude_bin_footprint', 0.36)
        self.declare_parameter('exclude_bin_z_max', 0.12)

        config_path = os.path.expanduser(
            self.get_parameter('config_path').get_parameter_value().string_value
        )
        repo_path = os.path.expanduser(
            self.get_parameter('icgnet_repo_path').get_parameter_value().string_value
        )
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.voxel_size = self.get_parameter('voxel_size').get_parameter_value().double_value
        self.n_grasps = self.get_parameter('n_grasps').get_parameter_value().integer_value
        self.workspace_bounds = {
            'x': (self.get_parameter('workspace_x_min').get_parameter_value().double_value,
                  self.get_parameter('workspace_x_max').get_parameter_value().double_value),
            'y': (self.get_parameter('workspace_y_min').get_parameter_value().double_value,
                  self.get_parameter('workspace_y_max').get_parameter_value().double_value),
            'z': (self.get_parameter('workspace_z_min').get_parameter_value().double_value,
                  self.get_parameter('workspace_z_max').get_parameter_value().double_value),
        }
        self._pc_config = PointCloudConfig(
            voxel_size=self.voxel_size,
            nb_neighbors=self.get_parameter('pc_nb_neighbors').get_parameter_value().integer_value,
            std_ratio=self.get_parameter('pc_std_ratio').get_parameter_value().double_value,
            normal_radius_factor=self.get_parameter('pc_normal_radius_factor').get_parameter_value().double_value,
            normal_max_nn=self.get_parameter('pc_normal_max_nn').get_parameter_value().integer_value,
        )
        self._exclude_bin = self.get_parameter('exclude_bin').get_parameter_value().bool_value
        self._bin_x = self.get_parameter('exclude_bin_x').get_parameter_value().double_value
        self._bin_y = self.get_parameter('exclude_bin_y').get_parameter_value().double_value
        self._bin_footprint = self.get_parameter('exclude_bin_footprint').get_parameter_value().double_value
        self._bin_z_max = self.get_parameter('exclude_bin_z_max').get_parameter_value().double_value

        # ── Model loading (non-fatal: node stays alive without model for debug) ─
        self.predictor = None
        if not config_path or not repo_path:
            self.get_logger().error(
                "config_path and/or icgnet_repo_path not set. "
                "Edit src/icgnet_main/config/icgnet_params.yaml"
            )
        else:
            try:
                self.predictor = ICGNetPredictor(config_path, icgnet_repo_path=repo_path)
                self.get_logger().info("ICGNet loaded successfully.")
            except Exception as e:
                self.get_logger().error(f"Failed to load ICGNet: {e}")

        # ── TF ───────────────────────────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Pointcloud subscriber (BEST_EFFORT — required for Gazebo sensor QoS) ─
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.latest_pc_msg = None
        self.create_subscription(PointCloud2, camera_topic, self._pc_callback, qos_sensor)

        # ── Publisher ────────────────────────────────────────────────────────
        self.grasp_pub = self.create_publisher(PoseArray, '/icgnet/grasps', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/icgnet/grasps_markers', 10)
        self.preprocessed_cloud_pub = self.create_publisher(
            PointCloud2, '/icgnet/preprocessed_cloud', 10
        )
        self.rich_pub = self.create_publisher(GraspArray, '/icgnet/grasps_rich', 10)
        # Raw per-instance reconstruction meshes (marching cubes, pre-hull/pre-clip) as
        # TRIANGLE_LIST markers — lets RViz show exactly what ICGNet reconstructs/segments.
        # TRANSIENT_LOCAL so a late-joining RViz still gets the last meshes.
        recon_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._recon_marker_pub = self.create_publisher(
            MarkerArray, '/icgnet/reconstruction_meshes', recon_qos
        )

        # ── Collision object publisher (MoveIt2 planning scene) ──────────────
        self._publish_co = self.get_parameter('publish_collision_objects').get_parameter_value().bool_value
        co_topic = self.get_parameter('collision_object_topic').get_parameter_value().string_value
        self._collision_pub = self.create_publisher(
            CollisionObject,
            co_topic,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            ),
        )
        self._published_collision_ids: set = set()

        # ── Service ──────────────────────────────────────────────────────────
        self.create_service(Trigger, '/icgnet/compute_grasps', self._compute_grasps_cb)

        self.get_logger().info(
            f"ICGNetGraspNode ready — topic={camera_topic}, "
            f"target_frame={self.target_frame}, n_grasps={self.n_grasps}"
        )

    def _pc_callback(self, msg: PointCloud2):
        self.latest_pc_msg = msg

    def _trimesh_to_collision_object(self, mesh, co_id: str, frame_id: str) -> CollisionObject:
        """Convert a trimesh.Trimesh (in world frame) to a MoveIt2 CollisionObject ADD message."""
        co = CollisionObject()
        co.header.frame_id = frame_id
        co.header.stamp = self.get_clock().now().to_msg()
        co.id = co_id
        sm = Mesh()
        sm.triangles = [
            MeshTriangle(vertex_indices=[int(f[0]), int(f[1]), int(f[2])])
            for f in mesh.faces
        ]
        sm.vertices = [
            Point(x=float(v[0]), y=float(v[1]), z=float(v[2]))
            for v in mesh.vertices
        ]
        co.meshes = [sm]
        identity_pose = Pose()
        identity_pose.orientation.w = 1.0
        co.mesh_poses = [identity_pose]
        co.operation = CollisionObject.ADD
        return co

    def _clip_mesh_exclude_bin(self, mesh):
        """Remove faces whose vertices fall inside the drop-bin AABB. Returns None if mesh is empty."""
        import trimesh
        half = self._bin_footprint / 2.0
        v = mesh.vertices
        in_bin = (
            (v[:, 0] >= self._bin_x - half) &
            (v[:, 0] <= self._bin_x + half) &
            (v[:, 1] >= self._bin_y - half) &
            (v[:, 1] <= self._bin_y + half) &
            (v[:, 2] <= self._bin_z_max)
        )
        bad = set(np.where(in_bin)[0])
        if not bad:
            return mesh
        good_faces = [f for f in mesh.faces if not (bad & set(f))]
        if not good_faces:
            return None
        clipped = trimesh.Trimesh(vertices=v, faces=np.array(good_faces), process=False)
        clipped.remove_unreferenced_vertices()
        return clipped

    def _publish_collision_objects_from_reconstructions(self, reconstructions: list, frame_id: str):
        """
        Remove previously published collision objects, then publish one per ICGNet instance.
        reconstructions: list of (trimesh.Trimesh, instance_id) tuples or just trimesh.Trimesh.
        """
        use_hull = self.get_parameter('collision_use_convex_hull').get_parameter_value().bool_value
        prefix = self.get_parameter('collision_id_prefix').get_parameter_value().string_value
        now = self.get_clock().now().to_msg()

        # Remove previously published instances
        for old_id in self._published_collision_ids:
            co = CollisionObject()
            co.header.frame_id = frame_id
            co.header.stamp = now
            co.id = old_id
            co.operation = CollisionObject.REMOVE
            self._collision_pub.publish(co)
        self._published_collision_ids.clear()

        # [RECON_DIAG] Report reconstruction count (helps diagnose multi-object blob issues).
        # If len == 1 for a multi-object scene: Mask3D under-segmentation (upstream, not a code bug).
        # If len == N but AABBs overlap: bounds margin or convex-hull inflation.
        self.get_logger().info(f'[RECON_DIAG] {len(reconstructions)} reconstruction(s) from ICGNet.')
        _raw_bounds: list = []  # (inst_id, bounds np[2,3]) before clip/hull, for overlap check

        # Publish new instances
        for idx, item in enumerate(reconstructions):
            if isinstance(item, tuple):
                mesh, inst_id = item
            else:
                mesh, inst_id = item, idx

            if len(mesh.faces) > 0:
                _b, _c = mesh.bounds, mesh.centroid
                self.get_logger().info(
                    f'[RECON_DIAG] inst={inst_id}: {len(mesh.vertices)}v {len(mesh.faces)}f '
                    f'AABB=[{_b[0, 0]:.3f},{_b[0, 1]:.3f},{_b[0, 2]:.3f}]->'
                    f'[{_b[1, 0]:.3f},{_b[1, 1]:.3f},{_b[1, 2]:.3f}] '
                    f'centroid=({_c[0]:.3f},{_c[1]:.3f},{_c[2]:.3f})'
                )
                _raw_bounds.append((inst_id, mesh.bounds.copy()))

            if len(mesh.faces) == 0:
                self.get_logger().warn(f"Instance {inst_id}: empty mesh, skipping collision object.")
                continue

            if self._exclude_bin:
                mesh = self._clip_mesh_exclude_bin(mesh)
                if mesh is None or len(mesh.faces) == 0:
                    self.get_logger().warn(f"Instance {inst_id}: mesh entirely in bin region, skipping.")
                    continue

            if use_hull:
                try:
                    mesh = mesh.convex_hull
                except Exception as e:
                    self.get_logger().warn(f"Convex hull failed for instance {inst_id}: {e}. Using full mesh.")

            co_id = f"{prefix}{inst_id}"
            co = self._trimesh_to_collision_object(mesh, co_id, frame_id)
            self._collision_pub.publish(co)
            self._published_collision_ids.add(co_id)
            self.get_logger().info(
                f"Published CollisionObject '{co_id}' ({len(mesh.faces)} triangles, hull={use_hull})."
            )

        # Warn if any two raw instance AABBs overlap (sign of Mask3D under-segmentation or hull inflation).
        for i in range(len(_raw_bounds)):
            for j in range(i + 1, len(_raw_bounds)):
                id_a, b_a = _raw_bounds[i]
                id_b, b_b = _raw_bounds[j]
                if all(b_a[0, k] < b_b[1, k] and b_b[0, k] < b_a[1, k] for k in range(3)):
                    self.get_logger().warn(
                        f'[RECON_DIAG] AABB OVERLAP: inst {id_a} ↔ inst {id_b}. '
                        'Possible Mask3D under-segmentation or convex-hull inflation.'
                    )

    def _publish_reconstruction_markers(self, reconstructions: list, frame_id: str):
        """Publish raw per-instance reconstruction meshes as TRIANGLE_LIST markers.

        Shows the actual marching-cubes surface (no convex hull, no bin clip) so the
        segmentation quality is directly visible in RViz — one solid colour per instance.
        """
        # Distinct, semi-transparent colours per instance (RGBA), cycled.
        palette = [
            (0.90, 0.10, 0.10), (0.10, 0.70, 0.20), (0.10, 0.30, 0.90),
            (0.90, 0.70, 0.10), (0.70, 0.10, 0.80), (0.10, 0.80, 0.80),
        ]
        now = self.get_clock().now().to_msg()
        ma = MarkerArray()

        clear = Marker()
        clear.header.frame_id = frame_id
        clear.header.stamp = now
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        for idx, item in enumerate(reconstructions):
            mesh, inst_id = item if isinstance(item, tuple) else (item, idx)
            if len(mesh.faces) == 0:
                continue
            r, g, b = palette[idx % len(palette)]
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = now
            m.ns = 'icgnet_reconstruction'
            m.id = int(inst_id)
            m.type = Marker.TRIANGLE_LIST
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 1.0
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 0.85
            verts = mesh.vertices
            for f in mesh.faces:
                for vi in (int(f[0]), int(f[1]), int(f[2])):
                    v = verts[vi]
                    m.points.append(Point(x=float(v[0]), y=float(v[1]), z=float(v[2])))
            ma.markers.append(m)

        self._recon_marker_pub.publish(ma)
        self.get_logger().info(
            f"[RECON_VIZ] Published {len(ma.markers) - 1} reconstruction mesh marker(s) "
            f"on /icgnet/reconstruction_meshes."
        )

    def _compute_grasps_cb(self, _req, response):
        if self.predictor is None:
            response.success = False
            response.message = "ICGNet not initialized. Check config_path and icgnet_repo_path."
            return response

        if self.latest_pc_msg is None:
            response.success = False
            response.message = "No pointcloud received. Check that the simulation is running."
            return response

        self.get_logger().info("Starting grasp computation...")
        try:
            result = self._run_inference()
        except Exception as e:
            self.get_logger().error(f"Inference error: {e}")
            response.success = False
            response.message = f"Error: {e}"
            return response

        n_total = result
        response.success = True
        response.message = f"Published {n_total} grasps"
        self.get_logger().info(response.message)
        return response

    def _run_inference(self):
        """Run the full inference pipeline and publish PoseArray, GraspArray, and CollisionObjects."""
        # 1. Convert ROS message to numpy
        raw_points = pointcloud2_to_numpy(self.latest_pc_msg)
        cloud_frame = self.latest_pc_msg.header.frame_id
        if raw_points.shape[0] == 0:
            raise RuntimeError("Empty pointcloud")

        # 2. TF: cloud_frame → target_frame (world)
        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                self.target_frame,
                cloud_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0),
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            raise RuntimeError(f"TF lookup {cloud_frame}→{self.target_frame} failed: {e}")

        t = tf_stamped.transform.translation
        q = tf_stamped.transform.rotation
        rot_mat = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        translation = np.array([t.x, t.y, t.z])

        points_world = (rot_mat @ raw_points.T).T + translation
        camera_pos_world = translation

        if self._exclude_bin:
            half = self._bin_footprint / 2.0
            in_bin = (
                (points_world[:, 0] >= self._bin_x - half) &
                (points_world[:, 0] <= self._bin_x + half) &
                (points_world[:, 1] >= self._bin_y - half) &
                (points_world[:, 1] <= self._bin_y + half) &
                (points_world[:, 2] <= self._bin_z_max)
            )
            n_removed = int(in_bin.sum())
            if n_removed > 0:
                points_world = points_world[~in_bin]
                self.get_logger().info(f"Bin exclusion: removed {n_removed} points.")

        # Dense cloud → Mask3D encoder (segmentation needs density); sparse cloud → grasp
        # sampling. Mirrors the ICGNet reference demo (full cloud to encoder, downsample
        # only the grasp branch).
        seg_pts, seg_normals, grasp_pts, grasp_normals = process_point_cloud_dual(
            points_world,
            config=self._pc_config,
            camera_position=camera_pos_world,
            workspace_bounds=self.workspace_bounds,
        )
        if seg_pts.shape[0] < MIN_POINTS_FOR_INFERENCE:
            raise RuntimeError(f"Too few points after preprocessing: {seg_pts.shape[0]}")

        self.get_logger().info(
            f"Preprocessing: {raw_points.shape[0]} → seg={seg_pts.shape[0]} (encoder), "
            f"grasp={grasp_pts.shape[0]} (sampling) points"
        )

        # 3b. Publish the dense segmentation cloud for debugging in RViz (what Mask3D sees)
        cloud_msg = _numpy_to_pointcloud2(seg_pts, self.target_frame, self.get_clock().now().to_msg())
        self.preprocessed_cloud_pub.publish(cloud_msg)

        do_meshes = self._publish_co and self.get_parameter('return_meshes').get_parameter_value().bool_value
        output = self.predictor.predict(
            seg_pts, seg_normals,
            grasp_points=grasp_pts, grasp_normals=grasp_normals,
            n_grasps=self.n_grasps, return_meshes=do_meshes,
        )

        # 5. Extract fields from ModelPredOut
        # scene_grasp_poses: [rot(G,3,3), centers(G,3), scores(G,), widths(G,), inst_ids(G,)]
        rot_matrices = output.scene_grasp_poses[0].cpu().numpy()
        centers      = output.scene_grasp_poses[1].cpu().numpy()
        scores       = output.scene_grasp_poses[2].cpu().numpy()
        widths       = output.scene_grasp_poses[3].cpu().numpy()
        inst_ids     = output.scene_grasp_poses[4].cpu().numpy()

        # class_predictions must be shaped (N_instances,) when the icg_net patch
        # (embeddings.semseg) is active.  If it is shaped (N_points,) the patch has NOT
        # been applied to ~/icg_net — class labels will be silently wrong for every object.
        # We detect this and warn loudly so the issue is surfaced at first GPU run.
        cls_arr = np.array([], dtype=np.int64)
        try:
            cls_arr = output.class_predictions.cpu().numpy()
            n_unique_insts = int(np.unique(inst_ids).shape[0])
            if len(cls_arr) != n_unique_insts and n_unique_insts > 0:
                self.get_logger().warn(
                    f"[PATCH CHECK] class_predictions length={len(cls_arr)} != "
                    f"n_instances={n_unique_insts}. "
                    "The icg_net patch (embeddings.semseg) may not be applied to ~/icg_net — "
                    "semantic class labels will be incorrect for multi-object scenes."
                )
            sem_class_raw = np.array(
                [int(cls_arr[iid]) if iid < len(cls_arr) else 6 for iid in inst_ids],
                dtype=np.int32,
            )
        except Exception as e:
            self.get_logger().warn(f"Could not extract semantic_class: {e} — defaulting to 6 (other)")
            sem_class_raw = np.full(len(inst_ids), 6, dtype=np.int32)

        n_total = len(centers)

        # Log predicted semantic class per reconstructed instance (visible even when 0 grasps).
        if output.reconstructions:
            for item in output.reconstructions:
                mesh, inst_id = item if isinstance(item, tuple) else (item, 0)
                cls_id = int(cls_arr[inst_id]) if inst_id < len(cls_arr) else 6
                self.get_logger().info(
                    f"[RECON] inst_{inst_id} → class={CLASS_NAMES.get(cls_id, '?')} (id={cls_id})"
                )
            # Visualise raw reconstruction meshes in RViz (one colour per instance).
            self._publish_reconstruction_markers(output.reconstructions, self.target_frame)

        # Log per-instance classification summary.
        unique_insts = np.unique(inst_ids.astype(int))
        inst_summary = []
        for iid in unique_insts:
            cls_id = int(sem_class_raw[inst_ids == iid][0])
            n_grasps_inst = int((inst_ids == iid).sum())
            inst_summary.append(f"inst_{iid}={CLASS_NAMES.get(cls_id, '?')}(id={cls_id}, {n_grasps_inst}g)")
        self.get_logger().info(
            f"[INSTANCES] {len(unique_insts)} instance(s): {', '.join(inst_summary)} | total_grasps={n_total}"
        )

        # Log per-instance grasp position spread (world frame) — reveals whether grasps
        # tagged for a class are physically clustered on the right object or scattered.
        for iid in unique_insts:
            c = centers[inst_ids == iid]
            if len(c) == 0:
                continue
            cls_id = int(sem_class_raw[inst_ids == iid][0])
            mn, mx, mean = c.min(axis=0), c.max(axis=0), c.mean(axis=0)
            self.get_logger().info(
                f"[GRASP_POS] inst_{iid} {CLASS_NAMES.get(cls_id, '?')} ({len(c)}g): "
                f"mean=({mean[0]:.3f},{mean[1]:.3f},{mean[2]:.3f}) "
                f"x=[{mn[0]:.3f},{mx[0]:.3f}] y=[{mn[1]:.3f},{mx[1]:.3f}] z=[{mn[2]:.3f},{mx[2]:.3f}]"
            )

        if n_total > 0:
            s_sorted = np.sort(scores)[::-1]
            top_k = min(10, n_total)
            self.get_logger().info(
                f"[SCORES] top-{top_k}: {[round(float(s), 4) for s in s_sorted[:top_k]]} | "
                f"min={s_sorted[-1]:.4f} max={s_sorted[0]:.4f} mean={scores.mean():.4f} | "
                f">0.3: {int((scores > 0.3).sum())} "
                f">0.5: {int((scores > 0.5).sum())} "
                f">0.7: {int((scores > 0.7).sum())}"
            )

        rot_filtered     = rot_matrices
        centers_filtered = centers
        scores_filtered  = scores
        widths_filtered  = widths
        inst_filtered    = inst_ids
        cls_filtered     = sem_class_raw

        now = self.get_clock().now().to_msg()

        pose_array = PoseArray()
        pose_array.header.frame_id = self.target_frame
        pose_array.header.stamp = now
        for i in range(len(centers_filtered)):
            p = Pose()
            p.position.x = float(centers_filtered[i, 0])
            p.position.y = float(centers_filtered[i, 1])
            p.position.z = float(centers_filtered[i, 2])
            quat = Rotation.from_matrix(rot_filtered[i]).as_quat()
            p.orientation.x = float(quat[0])
            p.orientation.y = float(quat[1])
            p.orientation.z = float(quat[2])
            p.orientation.w = float(quat[3])
            pose_array.poses.append(p)
        self.grasp_pub.publish(pose_array)

        marker_array = _build_grasp_markers(
            centers_filtered, rot_filtered, scores_filtered, widths_filtered,
            self.target_frame, now,
        )
        self.marker_pub.publish(marker_array)

        ga = GraspArray()
        ga.header.frame_id = self.target_frame
        ga.header.stamp = now
        for i in range(len(centers_filtered)):
            g = Grasp()
            g.pose = pose_array.poses[i]
            g.score = float(scores_filtered[i])
            g.width = float(widths_filtered[i])
            g.instance_id = int(inst_filtered[i])
            g.semantic_class = int(cls_filtered[i])
            ga.grasps.append(g)
        self.rich_pub.publish(ga)

        if self._publish_co and do_meshes and output.reconstructions:
            self._publish_collision_objects_from_reconstructions(output.reconstructions, self.target_frame)
            self.get_logger().info(
                f"Published {len(self._published_collision_ids)} collision object(s) to MoveIt2."
            )

        return n_total


def main(args=None):
    rclpy.init(args=args)
    node = ICGNetGraspNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
