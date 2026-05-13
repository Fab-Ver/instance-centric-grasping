import numpy as np
import open3d as o3d
import torch

def pointcloud2_to_numpy(msg):
    """
    Converte un messaggio sensor_msgs/PointCloud2 in un array numpy (N, 3).
    In un nodo ROS2 si consiglia l'uso di 'read_points' da 'sensor_msgs_py.point_cloud2'.
    """
    if isinstance(msg, np.ndarray):
        return msg
        
    # Fallback manuale per estrazione XYZ se la libreria non è disponibile
    # Assumiamo che i primi 3 campi siano x, y, z in float32
    try:
        from sensor_msgs_py import point_cloud2
        points_list = []
        for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points_list.append([p[0], p[1], p[2]])
        return np.array(points_list, dtype=np.float32)
    except ImportError:
        # Estrazione raw se sensor_msgs_py manca (meno robusto ma utile per test)
        fmt = np.float32
        data = np.frombuffer(msg.data, dtype=fmt)
        # Calcoliamo il numero di float per punto
        floats_per_point = msg.point_step // 4
        data = data.reshape(-1, floats_per_point)
        return data[:, :3].copy()

def process_point_cloud(points_np, voxel_size=0.01, nb_neighbors=20, std_ratio=2.0):
    """
    Preprocessa la point cloud per ICGNet:
    - Downsampling (Voxel Grid)
    - Rimozione Outlier
    - Stima delle Normali (richieste dal modello)
    """
    if points_np.shape[0] == 0:
        return np.array([]), np.array([])

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_np)
    
    # 1. Voxel Downsampling per migliorare le performance e uniformare la densità
    pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    
    # 2. Pulizia rumore statistico
    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    
    # 3. Calcolo Normali
    # ICGNet necessita delle normali per orientare i grasp correttamente
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size*5, max_nn=30))
    # Orienta le normali verso la camera (assumendo Z positivo verso l'alto/camera)
    pcd.orient_normals_to_align_with_direction(orientation_reference=np.array([0., 0., 1.]))
    
    points = np.asarray(pcd.points)
    normals = np.asarray(pcd.normals)
    
    return points, normals

def to_torch_tensors(points, normals, device='cuda'):
    """
    Converte gli array numpy in tensori PyTorch pronti per l'input del modello.
    """
    pts_t = torch.from_numpy(points).float().to(device)
    nrm_t = torch.from_numpy(normals).float().to(device)
    return pts_t, nrm_t
