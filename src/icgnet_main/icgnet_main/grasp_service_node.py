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

try:
    from icgnet_msgs.msg import Grasp, GraspArray
    _ICGNET_MSGS_AVAILABLE = True
except ImportError:
    _ICGNET_MSGS_AVAILABLE = False

try:
    from .icgnet_inference import ICGNetPredictor
    from .pointcloud_utils import (
        PointCloudConfig, gripper_keypoints_world, pointcloud2_to_numpy, process_point_cloud,
    )
except ImportError:
    import sys
    sys.path.append(os.path.dirname(__file__))
    from icgnet_inference import ICGNetPredictor
    from pointcloud_utils import (
        PointCloudConfig, gripper_keypoints_world, pointcloud2_to_numpy, process_point_cloud,
    )


def _score_to_color(score: float) -> ColorRGBA:
    """Map score [0,1] to HSV color: red=low, green=high."""
    h = max(0.0, min(1.0, score)) * 0.33
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
        self.declare_parameter('voxel_size', 0.01)
        self.declare_parameter('n_grasps', 32)
        self.declare_parameter('score_threshold', 0.0)
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
        self.declare_parameter('table_z_top', 0.05)
        self.declare_parameter('table_z_margin', 0.005)
        self.declare_parameter('pc_nb_neighbors', 20)
        self.declare_parameter('pc_std_ratio', 2.0)
        self.declare_parameter('pc_normal_radius_factor', 5.0)
        self.declare_parameter('pc_normal_max_nn', 30)

        config_path = os.path.expanduser(
            self.get_parameter('config_path').get_parameter_value().string_value
        )
        repo_path = os.path.expanduser(
            self.get_parameter('icgnet_repo_path').get_parameter_value().string_value
        )
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.voxel_size = self.get_parameter('voxel_size').get_parameter_value().double_value
        self.n_grasps = self.get_parameter('n_grasps').get_parameter_value().integer_value
        self.score_threshold = self.get_parameter('score_threshold').get_parameter_value().double_value
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
        self.rich_pub = (
            self.create_publisher(GraspArray, '/icgnet/grasps_rich', 10)
            if _ICGNET_MSGS_AVAILABLE else None
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

        # Publish new instances
        for idx, item in enumerate(reconstructions):
            if isinstance(item, tuple):
                mesh, inst_id = item
            else:
                mesh, inst_id = item, idx

            if len(mesh.faces) == 0:
                self.get_logger().warn(f"Instance {inst_id}: empty mesh, skipping collision object.")
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

        n_total, n_filtered = result
        response.success = True
        response.message = (
            f"Published {n_filtered} grasps "
            f"({n_total} total, score>={self.score_threshold:.2f})"
        )
        self.get_logger().info(response.message)
        return response

    def _run_inference(self):
        """
        Full pipeline:
        1. PointCloud2 → numpy (camera frame)
        2. TF: camera → world
        3. Preprocessing (voxel downsample + normals toward camera)
        4. ICGNet inference
        5. Publish PoseArray + MarkerArray + GraspArray
        """
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

        # world_pts = R @ cam_pts.T + t
        points_world = (rot_mat @ raw_points.T).T + translation
        camera_pos_world = translation

        # 3a. Remove table surface points (z ≤ table_z_top + table_z_margin)
        table_z_top = self.get_parameter('table_z_top').get_parameter_value().double_value
        table_z_margin = self.get_parameter('table_z_margin').get_parameter_value().double_value
        z_cutoff = table_z_top + table_z_margin
        above_table = points_world[:, 2] > z_cutoff
        n_removed = int((~above_table).sum())
        points_world = points_world[above_table]
        if n_removed > 0:
            self.get_logger().info(f"Table filter: removed {n_removed} pts at z≤{z_cutoff:.3f}m")

        # 3. Preprocessing (crop → downsample → outlier removal → normals)
        pts, normals = process_point_cloud(
            points_world,
            config=self._pc_config,
            camera_position=camera_pos_world,
            workspace_bounds=self.workspace_bounds,
        )
        if pts.shape[0] < 50:
            raise RuntimeError(f"Too few points after preprocessing: {pts.shape[0]}")

        self.get_logger().info(f"Preprocessing: {raw_points.shape[0]} → {pts.shape[0]} points")

        # 3b. Publish preprocessed cloud for debugging in RViz
        cloud_msg = _numpy_to_pointcloud2(pts, self.target_frame, self.get_clock().now().to_msg())
        self.preprocessed_cloud_pub.publish(cloud_msg)

        # 4. ICGNet inference
        do_meshes = self._publish_co and self.get_parameter('return_meshes').get_parameter_value().bool_value
        output = self.predictor.predict(pts, normals, n_grasps=self.n_grasps, return_meshes=do_meshes)

        # 5. Extract fields from ModelPredOut
        # scene_grasp_poses: [rot(G,3,3), centers(G,3), scores(G,), widths(G,), inst_ids(G,)]
        rot_matrices = output.scene_grasp_poses[0].cpu().numpy()
        centers      = output.scene_grasp_poses[1].cpu().numpy()
        scores       = output.scene_grasp_poses[2].cpu().numpy()
        widths       = output.scene_grasp_poses[3].cpu().numpy()
        inst_ids     = output.scene_grasp_poses[4].cpu().numpy()

        # class_predictions has shape (N_points,) — per-point semantic labels.
        # We index it with inst_id as a best-effort mapping: works when
        # class_predictions is actually (N_instance_queries,). If the shapes
        # don't match, we fall back to class 6 (other) and log a warning.
        try:
            cls_arr = output.class_predictions.cpu().numpy()
            sem_class_raw = np.array(
                [int(cls_arr[iid]) if iid < len(cls_arr) else 6 for iid in inst_ids],
                dtype=np.int32,
            )
        except Exception as e:
            self.get_logger().warn(f"Could not extract semantic_class: {e} — defaulting to 6 (other)")
            sem_class_raw = np.full(len(inst_ids), 6, dtype=np.int32)

        n_total = len(centers)

        # 6. Filter by score
        mask = scores >= self.score_threshold
        rot_f     = rot_matrices[mask]
        centers_f = centers[mask]
        scores_f  = scores[mask]
        widths_f  = widths[mask]
        inst_f    = inst_ids[mask]
        cls_f     = sem_class_raw[mask]

        now = self.get_clock().now().to_msg()

        # 7. Publish PoseArray
        pose_array = PoseArray()
        pose_array.header.frame_id = self.target_frame
        pose_array.header.stamp = now
        for i in range(len(centers_f)):
            p = Pose()
            p.position.x = float(centers_f[i, 0])
            p.position.y = float(centers_f[i, 1])
            p.position.z = float(centers_f[i, 2])
            quat = Rotation.from_matrix(rot_f[i]).as_quat()
            p.orientation.x = float(quat[0])
            p.orientation.y = float(quat[1])
            p.orientation.z = float(quat[2])
            p.orientation.w = float(quat[3])
            pose_array.poses.append(p)
        self.grasp_pub.publish(pose_array)

        marker_array = _build_grasp_markers(centers_f, rot_f, scores_f, widths_f, self.target_frame, now)
        self.marker_pub.publish(marker_array)

        # 9. Publish GraspArray with full metadata (consumed by grasp_executor)
        if self.rich_pub is not None:
            ga = GraspArray()
            ga.header.frame_id = self.target_frame
            ga.header.stamp = now
            for i in range(len(centers_f)):
                g = Grasp()
                g.pose = pose_array.poses[i]
                g.score = float(scores_f[i])
                g.width = float(widths_f[i])
                g.instance_id = int(inst_f[i])
                g.semantic_class = int(cls_f[i])
                ga.grasps.append(g)
            self.rich_pub.publish(ga)

        # 10. Publish CollisionObjects from ICGNet reconstructions
        if self._publish_co and do_meshes and output.reconstructions:
            self._publish_collision_objects_from_reconstructions(output.reconstructions, self.target_frame)
            self.get_logger().info(
                f"Published {len(self._published_collision_ids)} collision object(s) to MoveIt2."
            )

        return n_total, mask.sum()


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
