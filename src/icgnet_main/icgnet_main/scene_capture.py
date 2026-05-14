import threading
import pathlib

import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger
import tf2_ros

try:
    from .pointcloud_utils import pointcloud2_to_numpy
except ImportError:
    from pointcloud_utils import pointcloud2_to_numpy


class SceneCaptureNode(Node):
    def __init__(self):
        super().__init__('scene_capture')

        self._latest_cloud = None
        self._lock = threading.Lock()
        self._scene_counter = 0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(PointCloud2, '/camera/rgbd_camera/points',
                                 self._cloud_cb, qos)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_service(Trigger, '/icgnet/capture_scene', self._capture_cb)
        self.get_logger().info('SceneCaptureNode pronto — chiama /icgnet/capture_scene')

    def _cloud_cb(self, msg):
        with self._lock:
            self._latest_cloud = msg

    def _capture_cb(self, _req, response):
        with self._lock:
            cloud_msg = self._latest_cloud

        if cloud_msg is None:
            response.success = False
            response.message = 'Nessuna pointcloud ricevuta su /camera/rgbd_camera/points'
            return response

        try:
            tf = self._tf_buffer.lookup_transform(
                'world', 'camera_link_optical',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0),
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            response.success = False
            response.message = f'TF lookup fallito: {e}'
            return response

        points = pointcloud2_to_numpy(cloud_msg)
        if points.shape[0] == 0:
            response.success = False
            response.message = 'Pointcloud vuota'
            return response

        t = tf.transform.translation
        q = tf.transform.rotation
        rot = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        translation = np.array([t.x, t.y, t.z])

        # Trasforma: world_pts = R @ cam_pts.T + t (broadcast su N punti)
        points_world = (rot @ points.T).T + translation

        output_dir = pathlib.Path('/tmp/scenes')
        output_dir.mkdir(parents=True, exist_ok=True)

        self._scene_counter += 1
        scene_id = f'scene_{self._scene_counter:03d}'
        out_path = output_dir / f'{scene_id}.pcd'

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_world.astype(np.float64))
        o3d.io.write_point_cloud(str(out_path), pcd)

        msg_out = f'Salvati {len(points_world)} punti → {out_path}'
        self.get_logger().info(msg_out)
        response.success = True
        response.message = msg_out
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SceneCaptureNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
