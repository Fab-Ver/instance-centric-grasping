import colorsys
import os

import numpy as np
import rclpy
import rclpy.duration
import rclpy.time
import tf2_ros
from geometry_msgs.msg import Point, PoseArray, Pose
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

try:
    from .icgnet_inference import ICGNetPredictor
    from .pointcloud_utils import pointcloud2_to_numpy, process_point_cloud
except ImportError:
    import sys
    sys.path.append(os.path.dirname(__file__))
    from icgnet_inference import ICGNetPredictor
    from pointcloud_utils import pointcloud2_to_numpy, process_point_cloud


def _score_to_rgb(score: float):
    """Mappa score [0,1] su colore HSV: rosso=basso, verde=alto."""
    h = max(0.0, min(1.0, score)) * 0.33   # hue 0° (red) → 120° (green)
    r, g, b = colorsys.hsv_to_rgb(h, 1.0, 1.0)
    return float(r), float(g), float(b)


def _build_grasp_markers(centers, rot_matrices, scores, frame_id, now):
    """
    Costruisce un MarkerArray con una freccia per ogni grasp.
    La freccia parte dal centro del grasp e punta lungo l'asse di approach (z della rotazione).
    Il colore è mappato sullo score: rosso=basso, verde=alto.
    """
    ma = MarkerArray()

    # Marker DELETE_ALL per ripulire i grasp precedenti
    clear = Marker()
    clear.header.frame_id = frame_id
    clear.header.stamp = now
    clear.action = Marker.DELETEALL
    ma.markers.append(clear)

    for i, (c, R, s) in enumerate(zip(centers, rot_matrices, scores)):
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = now
        m.ns = "icgnet_grasps"
        m.id = i
        m.type = Marker.ARROW
        m.action = Marker.ADD

        # Direzione di approach = z-axis della matrice di rotazione
        approach = R[:, 2]
        length = 0.07  # 7 cm
        start = Point(x=float(c[0]), y=float(c[1]), z=float(c[2]))
        end = Point(
            x=float(c[0] + length * approach[0]),
            y=float(c[1] + length * approach[1]),
            z=float(c[2] + length * approach[2]),
        )
        m.points = [start, end]
        m.scale.x = 0.006   # diametro gambo
        m.scale.y = 0.012   # diametro testa

        r, g, b = _score_to_rgb(float(s))
        m.color = ColorRGBA(r=r, g=g, b=b, a=0.85)
        m.lifetime = rclpy.duration.Duration(seconds=60).to_msg()
        ma.markers.append(m)

    return ma


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

        # ── Caricamento modello (non-fatale: il nodo resta attivo per debug) ─
        self.predictor = None
        if not config_path or not repo_path:
            self.get_logger().error(
                "config_path e/o icgnet_repo_path non configurati. "
                "Modifica src/icgnet_main/config/icgnet_params.yaml"
            )
        else:
            try:
                self.predictor = ICGNetPredictor(config_path, icgnet_repo_path=repo_path)
                self.get_logger().info("ICGNet caricato correttamente.")
            except Exception as e:
                self.get_logger().error(f"Impossibile caricare ICGNet: {e}")

        # ── TF ───────────────────────────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Subscriber pointcloud (BEST_EFFORT per compatibilità con Gazebo) ─
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

        # ── Servizio ─────────────────────────────────────────────────────────
        self.create_service(Trigger, '/icgnet/compute_grasps', self._compute_grasps_cb)

        self.get_logger().info(
            f"ICGNetGraspNode pronto — topic={camera_topic}, "
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
            response.message = "Nessuna pointcloud ricevuta. Verifica che la simulazione sia attiva."
            return response

        self.get_logger().info("Avvio calcolo grasp...")
        try:
            result = self._run_inference()
        except Exception as e:
            self.get_logger().error(f"Errore durante l'inferenza: {e}")
            response.success = False
            response.message = f"Errore: {e}"
            return response

        n_total, n_filtered = result
        response.success = True
        response.message = (
            f"{n_filtered} grasp pubblicati "
            f"({n_total} totali, soglia score>={self.score_threshold:.2f})"
        )
        self.get_logger().info(response.message)
        return response

    def _run_inference(self):
        """
        Pipeline completa:
        1. PointCloud2 → numpy (frame camera)
        2. TF: camera → world
        3. Preprocessing (voxel + normali verso camera)
        4. ICGNet inference
        5. Pubblica PoseArray + MarkerArray
        """
        # 1. Converti messaggio in numpy
        raw_points = pointcloud2_to_numpy(self.latest_pc_msg)
        cloud_frame = self.latest_pc_msg.header.frame_id
        if raw_points.shape[0] == 0:
            raise RuntimeError("Pointcloud vuota")

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
            raise RuntimeError(f"TF lookup {cloud_frame}→{self.target_frame} fallito: {e}")

        t = tf_stamped.transform.translation
        q = tf_stamped.transform.rotation
        rot_mat = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        translation = np.array([t.x, t.y, t.z])

        # Trasforma: world_pts = R @ cam_pts.T + t
        points_world = (rot_mat @ raw_points.T).T + translation
        camera_pos_world = translation   # posizione camera nel frame world

        # 3. Preprocessing
        pts, normals = process_point_cloud(
            points_world,
            voxel_size=self.voxel_size,
            camera_position=camera_pos_world,
        )
        if pts.shape[0] < 50:
            raise RuntimeError(f"Punti insufficienti dopo preprocessing: {pts.shape[0]}")

        self.get_logger().info(f"Preprocessing: {raw_points.shape[0]} → {pts.shape[0]} punti")

        # 4. ICGNet inference
        output = self.predictor.predict(pts, normals, n_grasps=self.n_grasps)

        # 5. Estrai campi da ModelPredOut
        # scene_grasp_poses: [rot(G,3,3), centers(G,3), scores(G,), widths(G,), inst_ids(G,)]
        rot_matrices = output.scene_grasp_poses[0].cpu().numpy()
        centers      = output.scene_grasp_poses[1].cpu().numpy()
        scores       = output.scene_grasp_poses[2].cpu().numpy()
        widths       = output.scene_grasp_poses[3].cpu().numpy()
        inst_ids     = output.scene_grasp_poses[4].cpu().numpy()

        
        n_total = len(centers)

        # 6. Filtra per score
        mask = scores >= self.score_threshold
        rot_f     = rot_matrices[mask]
        centers_f = centers[mask]
        scores_f  = scores[mask]
        widths_f  = widths[mask]
        inst_f    = inst_ids[mask]
        #cls_f     = cls[mask] if len(cls) >= len(centers) else np.zeros(mask.sum(), dtype=int)

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

        # 8. Pubblica MarkerArray (frecce colorate per score)
        marker_array = _build_grasp_markers(
            centers_f, rot_f, scores_f, self.target_frame, now
        )
        self.marker_pub.publish(marker_array)

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
