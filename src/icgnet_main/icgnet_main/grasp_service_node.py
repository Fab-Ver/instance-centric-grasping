import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseArray, Pose
from std_srvs.srv import Trigger
import numpy as np
from scipy.spatial.transform import Rotation as R
from loguru import logger

# Import locali dal pacchetto
try:
    from .icgnet_inference import ICGNetPredictor
    from .pointcloud_utils import pointcloud2_to_numpy, process_point_cloud
except ImportError:
    # Per esecuzione diretta o test
    import sys
    import os
    sys.path.append(os.path.dirname(__file__))
    from icgnet_inference import ICGNetPredictor
    from pointcloud_utils import pointcloud2_to_numpy, process_point_cloud

class ICGNetGraspNode(Node):
    def __init__(self):
        super().__init__('icgnet_grasp_node')
        
        # Dichiarazione Parametri
        self.declare_parameter('config_path', 'path/to/config.yaml')
        self.declare_parameter('icgnet_repo_path', 'path/to/icg_net_repo')
        self.declare_parameter('camera_topic', '/camera/rgbd_camera/points')
        self.declare_parameter('voxel_size', 0.01)
        self.declare_parameter('n_grasps', 32)
        
        config_path = self.get_parameter('config_path').get_parameter_value().string_value
        repo_path = self.get_parameter('icgnet_repo_path').get_parameter_value().string_value
        self.n_grasps = self.get_parameter('n_grasps').get_parameter_value().integer_value
        self.voxel_size = self.get_parameter('voxel_size').get_parameter_value().double_value
        
        # Inizializzazione del predittore (carica il modello in GPU)
        try:
            self.predictor = ICGNetPredictor(config_path, icgnet_repo_path=repo_path)
        except Exception as e:
            self.get_logger().error(f"Impossibile inizializzare ICGNet: {e}")
            # Non solleviamo eccezione per permettere al nodo di restare attivo per debug
        
        # Buffer per l'ultima pointcloud ricevuta
        self.latest_pc_msg = None
        
        # Sottoscrizione alla PointCloud
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.pc_sub = self.create_subscription(
            PointCloud2,
            camera_topic,
            self.pc_callback,
            10
        )
        
        # Publisher per i risultati (Visualizzazione in RViz)
        self.grasp_pub = self.create_publisher(PoseArray, '/icgnet/grasps', 10)
        
        # Servizio per triggerare l'inferenza
        self.srv = self.create_service(Trigger, '/icgnet/compute_grasps', self.handle_compute_grasps)
        
        self.get_logger().info(f"Nodo ICGNet avviato. In ascolto su {camera_topic}")

    def pc_callback(self, msg):
        """Salva l'ultimo messaggio pointcloud ricevuto."""
        self.latest_pc_msg = msg

    def handle_compute_grasps(self, request, response):
        """Gestisce la richiesta di calcolo dei grasp."""
        if self.latest_pc_msg is None:
            response.success = False
            response.message = "Errore: Nessun dato ricevuto dalla camera."
            return response
            
        self.get_logger().info("Ricevuta richiesta di calcolo grasp. Elaborazione in corso...")
        
        try:
            # 1. Pre-processing
            raw_points = pointcloud2_to_numpy(self.latest_pc_msg)
            processed_points, normals = process_point_cloud(raw_points, voxel_size=self.voxel_size)
            
            if len(processed_points) < 50:
                response.success = False
                response.message = f"Errore: Punti insufficienti ({len(processed_points)}) per l'inferenza."
                return response

            # 2. Inferenza
            output = self.predictor.predict(processed_points, normals, n_grasps=self.n_grasps)
            
            # 3. Estrazione Pose
            # In base all'output di ICGNet (ModelPredOut):
            # scene_grasp_poses è una lista: [RotationMatrices, Centers, Scores, Widths, Labels]
            # RotationMatrices: (G, 3, 3), Centers: (G, 3)
            grasps_data = output.scene_grasp_poses 
            
            rot_matrices = grasps_data[0].cpu().numpy() # (G, 3, 3)
            centers = grasps_data[1].cpu().numpy()      # (G, 3)
            
            pose_array = PoseArray()
            pose_array.header.frame_id = self.latest_pc_msg.header.frame_id
            pose_array.header.stamp = self.get_clock().now().to_msg()
            
            for i in range(len(centers)):
                p = Pose()
                p.position.x = float(centers[i, 0])
                p.position.y = float(centers[i, 1])
                p.position.z = float(centers[i, 2])
                
                # Conversione matrice 3x3 -> Quaternione [x, y, z, w]
                r = R.from_matrix(rot_matrices[i])
                quat = r.as_quat() 
                
                p.orientation.x = float(quat[0])
                p.orientation.y = float(quat[1])
                p.orientation.z = float(quat[2])
                p.orientation.w = float(quat[3])
                pose_array.poses.append(p)
                
            # Pubblica i risultati per il debug/visualizzazione
            self.grasp_pub.publish(pose_array)
            
            response.success = True
            response.message = f"Calcolo completato: {len(pose_array.poses)} grasp pubblicati su /icgnet/grasps"
            self.get_logger().info(response.message)
            
        except Exception as e:
            self.get_logger().error(f"Errore durante l'inferenza: {e}")
            response.success = False
            response.message = f"Eccezione: {str(e)}"
            
        return response

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
