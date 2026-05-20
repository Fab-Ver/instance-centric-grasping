import colorsys
import os
import struct

import numpy as np
import rclpy
import rclpy.duration
import rclpy.time
import tf2_ros
from geometry_msgs.msg import Point, PoseArray, Pose
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2, PointField
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
    from .pointcloud_utils import pointcloud2_to_numpy, process_point_cloud
except ImportError:
    import sys
    sys.path.append(os.path.dirname(__file__))
    from icgnet_inference import ICGNetPredictor
    from pointcloud_utils import pointcloud2_to_numpy, process_point_cloud


def _score_to_color(score: float) -> ColorRGBA:
    """Map score [0,1] to HSV color: red=low, green=high."""
    h = max(0.0, min(1.0, score)) * 0.33
    r, g, b = colorsys.hsv_to_rgb(h, 1.0, 1.0)
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=0.9)


def _gripper_points_world(c: np.ndarray, R: np.ndarray, w: float):
    """
    Return the 6 keypoints of the gripper wireframe in world frame.
    Local frame: y=finger-closing (surface normal), z=approach (toward object).
    Gripper geometry matches ICGNet's create_our_gripper_marker with
    center_offset=[0,0,-0.045] and rotations=[0,0,pi/2].

    Segments (each as (start, end) pair):
      left-finger, right-finger, crossbar, handle
    """
    half_w = w / 2.0
    # Keypoints in local frame (TCP = origin)
    lf_base = np.array([0.0,  half_w, -0.045])
    rf_base = np.array([0.0, -half_w, -0.045])
    lf_tip  = np.array([0.0,  half_w,  0.005])
    rf_tip  = np.array([0.0, -half_w,  0.005])
    cb      = np.array([0.0,  0.0,    -0.045])   # crossbar center = handle start
    handle  = np.array([0.0,  0.0,    -0.115])   # handle end

    def wp(lp):
        return c + R @ lp

    return [
        (wp(lf_base), wp(lf_tip)),    # left finger
        (wp(rf_base), wp(rf_tip)),    # right finger
        (wp(rf_base), wp(lf_base)),   # crossbar
        (wp(cb),      wp(handle)),    # handle
    ]


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
        segments = _gripper_points_world(c, R, w_clipped)
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

        # ── Parametri ────────────────────────────────────────────────────────
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

        # ── Model loading (non-fatal: node stays alive without model for debug) ─
        self.predictor = None
        if not config_path or not repo_path:
            self.get_logger().error(
                "config_path e/o icgnet_repo_path non configurati. "
                "Modifica src/icgnet_main/config/icgnet_params.yaml"
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

        # ── Service ──────────────────────────────────────────────────────────
        self.create_service(Trigger, '/icgnet/compute_grasps', self._compute_grasps_cb)

        self.get_logger().info(
            f"ICGNetGraspNode ready — topic={camera_topic}, "
            f"target_frame={self.target_frame}, n_grasps={self.n_grasps}"
        )

    def _pc_callback(self, msg: PointCloud2):
        self.latest_pc_msg = msg

    def _compute_grasps_cb(self, _req, response):
        if self.predictor is None:
            response.success = False
            response.message = "ICGNet non inizializzato. Controlla config_path e icgnet_repo_path."
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

        # 3. Preprocessing (crop → downsample → outlier removal → normals)
        pts, normals = process_point_cloud(
            points_world,
            voxel_size=self.voxel_size,
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
        output = self.predictor.predict(pts, normals, n_grasps=self.n_grasps)

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

        # 6. Filtra per score
        mask = scores >= self.score_threshold
        rot_f     = rot_matrices[mask]
        centers_f = centers[mask]
        scores_f  = scores[mask]
        widths_f  = widths[mask]
        inst_f    = inst_ids[mask]
        cls_f     = sem_class_raw[mask]

        now = self.get_clock().now().to_msg()

        # 7. Pubblica PoseArray
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

        # ICGNet all-grasps markers disabled — only grasp_executor's current-grasp marker shown.
        # Uncomment to re-enable the full grasp heatmap in RViz.
        # marker_array = _build_grasp_markers(
        #     centers_f, rot_f, scores_f, widths_f, self.target_frame, now
        # )
        # self.marker_pub.publish(marker_array)

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
