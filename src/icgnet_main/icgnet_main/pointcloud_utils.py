from dataclasses import dataclass

import numpy as np
import open3d as o3d
import torch


@dataclass
class PointCloudConfig:
    voxel_size: float = 0.01
    nb_neighbors: int = 20
    std_ratio: float = 2.0
    normal_radius_factor: float = 5.0
    normal_max_nn: int = 30


def gripper_keypoints_world(
    center: np.ndarray,
    rot_mat: np.ndarray,
    width: float,
) -> list[tuple[np.ndarray, np.ndarray]]:
    """
    Return 4 line segments (start, end) of the gripper wireframe in world frame.

    Local frame: y=finger-closing (surface normal), z=approach (toward object).
    Geometry matches ICGNet's create_our_gripper_marker with
    center_offset=[0,0,-0.045] and rotations=[0,0,pi/2].
    """
    half_w = width / 2.0
    lf_base = np.array([0.0,  half_w, -0.045])
    rf_base = np.array([0.0, -half_w, -0.045])
    lf_tip  = np.array([0.0,  half_w,  0.005])
    rf_tip  = np.array([0.0, -half_w,  0.005])
    cb      = np.array([0.0,  0.0,    -0.045])
    handle  = np.array([0.0,  0.0,    -0.115])

    def to_world(local_pt: np.ndarray) -> np.ndarray:
        return center + rot_mat @ local_pt

    return [
        (to_world(lf_base), to_world(lf_tip)),
        (to_world(rf_base), to_world(rf_tip)),
        (to_world(rf_base), to_world(lf_base)),
        (to_world(cb),      to_world(handle)),
    ]


def pointcloud2_to_numpy(msg) -> np.ndarray:
    """
    Convert a sensor_msgs/PointCloud2 message to a numpy array (N, 3).
    Uses sensor_msgs_py if available; falls back to raw buffer extraction.
    """
    if isinstance(msg, np.ndarray):
        return msg

    try:
        from sensor_msgs_py import point_cloud2
        points_list = []
        for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points_list.append([p[0], p[1], p[2]])
        return np.array(points_list, dtype=np.float32)
    except ImportError:
        # Raw fallback — assumes first 3 fields are x, y, z as float32
        data = np.frombuffer(msg.data, dtype=np.float32)
        floats_per_point = msg.point_step // 4
        data = data.reshape(-1, floats_per_point)
        return data[:, :3].copy()


def crop_to_workspace(points_np: np.ndarray, bounds: dict) -> np.ndarray:
    """
    Remove points outside the workspace bounding box (world frame).

    bounds: dict with keys 'x', 'y', 'z', each a (min, max) tuple.
    Axes not present in bounds are left uncropped.
    """
    mask = np.ones(len(points_np), dtype=bool)
    for axis, idx in (('x', 0), ('y', 1), ('z', 2)):
        if axis in bounds:
            lo, hi = bounds[axis]
            mask &= (points_np[:, idx] >= lo) & (points_np[:, idx] <= hi)
    return points_np[mask]


def process_point_cloud(
    points_np: np.ndarray,
    config: PointCloudConfig,
    camera_position: np.ndarray,
    workspace_bounds: dict | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Preprocess the point cloud for ICGNet:
    crop → voxel downsample → statistical outlier removal → normals toward camera.
    """
    if points_np.shape[0] == 0:
        return np.array([]).reshape(0, 3), np.array([]).reshape(0, 3)

    if workspace_bounds is not None:
        points_np = crop_to_workspace(points_np, workspace_bounds)
        if points_np.shape[0] == 0:
            return np.array([]).reshape(0, 3), np.array([]).reshape(0, 3)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_np)
    pcd = pcd.voxel_down_sample(voxel_size=config.voxel_size)
    pcd, _ = pcd.remove_statistical_outlier(
        nb_neighbors=config.nb_neighbors,
        std_ratio=config.std_ratio,
    )
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=config.voxel_size * config.normal_radius_factor,
            max_nn=config.normal_max_nn,
        )
    )
    pcd.orient_normals_towards_camera_location(
        np.array(camera_position, dtype=np.float64)
    )

    return np.asarray(pcd.points), np.asarray(pcd.normals)


def to_torch_tensors(
    points: np.ndarray,
    normals: np.ndarray,
    device: str = 'cuda',
) -> tuple[torch.Tensor, torch.Tensor]:
    """Convert numpy arrays to PyTorch tensors for model input."""
    pts_t = torch.from_numpy(points).float().to(device)
    nrm_t = torch.from_numpy(normals).float().to(device)
    return pts_t, nrm_t
