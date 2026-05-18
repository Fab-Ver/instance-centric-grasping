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

def crop_to_workspace(points_np: np.ndarray, bounds: dict) -> np.ndarray:
    """
    Removes points outside the workspace bounding box (world frame).

    bounds: dict with keys 'x', 'y', 'z', each a (min, max) tuple.
    Axes not present in bounds are left uncropped.
    """
    mask = np.ones(len(points_np), dtype=bool)
    for axis, idx in (('x', 0), ('y', 1), ('z', 2)):
        if axis in bounds:
            lo, hi = bounds[axis]
            mask &= (points_np[:, idx] >= lo) & (points_np[:, idx] <= hi)
    return points_np[mask]


def process_point_cloud(points_np, voxel_size=0.01, nb_neighbors=20, std_ratio=2.0,
                        camera_position=(0.0, 0.0, 0.0), workspace_bounds=None):
    """
    Preprocesses the point cloud for ICGNet:
    - Workspace crop (optional): removes robot body, table surface, out-of-range points
    - Voxel downsampling
    - Statistical outlier removal
    - Normal estimation oriented toward camera (required by ICGNet)

    Args:
        camera_position: camera position in the cloud frame (3D tuple/array).
        workspace_bounds: optional dict {'x': (min, max), 'y': (min, max), 'z': (min, max)}
                          in world frame. Points outside are discarded before downsampling.
    """
    if points_np.shape[0] == 0:
        return np.array([]).reshape(0, 3), np.array([]).reshape(0, 3)

    # Crop to workspace before downsampling — cheaper and removes spurious grasps
    # on robot body and table surface.
    if workspace_bounds is not None:
        points_np = crop_to_workspace(points_np, workspace_bounds)
        if points_np.shape[0] == 0:
            return np.array([]).reshape(0, 3), np.array([]).reshape(0, 3)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_np)

    # 1. Voxel Downsampling
    pcd = pcd.voxel_down_sample(voxel_size=voxel_size)

    # 2. Pulizia rumore statistico
    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)

    # 3. Stima normali + orientamento verso la camera
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=30)
    )
    pcd.orient_normals_towards_camera_location(
        np.array(camera_position, dtype=np.float64)
    )

    return np.asarray(pcd.points), np.asarray(pcd.normals)

def to_torch_tensors(points, normals, device='cuda'):
    """
    Converte gli array numpy in tensori PyTorch pronti per l'input del modello.
    """
    pts_t = torch.from_numpy(points).float().to(device)
    nrm_t = torch.from_numpy(normals).float().to(device)
    return pts_t, nrm_t
