import sys
import os
from typing import Any

import numpy as np
import torch
from rclpy.logging import get_logger

_logger = get_logger('icgnet_predictor')


class ICGNetPredictor:
    def __init__(self, config_path: str, icgnet_repo_path: str | None = None, device: str = 'cuda') -> None:
        """
        Load ICGNet model from checkpoint.

        :param config_path: Absolute path to the model config.yaml.
        :param icgnet_repo_path: Root of the cloned icg_net repository.
        :param device: 'cuda' or 'cpu'.
        """
        self.device = device if torch.cuda.is_available() else 'cpu'

        if icgnet_repo_path:
            abs_repo_path = os.path.abspath(icgnet_repo_path)
            if abs_repo_path not in sys.path:
                sys.path.insert(0, abs_repo_path)
                _logger.info(f"Added {abs_repo_path} to sys.path")

        try:
            from icg_net import get_model
            _logger.info(f"Loading model from: {config_path}")
            self.model = get_model(config_path, device=self.device)
            self.model.eval()
            _logger.info(f"ICGNet loaded successfully on {self.device}")
        except ImportError as e:
            _logger.error(f"Failed to import 'icg_net': {e}")
            _logger.error("Make sure icg_net is cloned and icgnet_repo_path is correct.")
            raise
        except Exception as e:
            _logger.error(f"Error loading model: {e}")
            raise

    def predict(
        self,
        points: np.ndarray,
        normals: np.ndarray,
        grasp_points: np.ndarray | None = None,
        grasp_normals: np.ndarray | None = None,
        n_grasps: int = 64,
        return_meshes: bool = False,
        score_threshold: float = 0.0,
    ) -> Any:
        """
        Run inference on the given point cloud.

        :param points: numpy array (N, 3) — dense cloud fed to the Mask3D encoder
                       (segmentation + reconstruction). Density matters for instance
                       separation, so this should NOT be voxel-downsampled.
        :param normals: numpy array (N, 3) of normals for ``points``.
        :param grasp_points: numpy array (M, 3) — sparse cloud used only for grasp-point
                       sampling. Defaults to ``points`` if None.
        :param grasp_normals: normals for ``grasp_points`` (defaults to ``normals``).
        :param n_grasps: number of grasps to generate.
        :param return_meshes: if True, run marching cubes per instance and populate
                              ModelPredOut.reconstructions (list of trimesh.Trimesh).
                              Adds ~0.5-2s latency on GPU. Use on-demand only.
        :param score_threshold: ICGNet drops grasps with score <= this before returning.
                              Default 0.0 disables the cull (the model's own default is 0.3),
                              so every candidate is returned and ranked by the executor — no
                              grasp is excluded upstream, low-score ones are simply tried last.
        :return: ModelPredOut with scene_grasp_poses, class_predictions, reconstructions.
        """
        from .pointcloud_utils import to_torch_tensors

        pts_t, nrm_t = to_torch_tensors(points, normals, device=self.device)
        if grasp_points is None:
            grasp_pts_t, grasp_nrm_t = pts_t, nrm_t
        else:
            grasp_pts_t, grasp_nrm_t = to_torch_tensors(
                grasp_points, grasp_normals, device=self.device
            )

        _logger.info(f"Running inference on {pts_t.shape[0]} points (return_meshes={return_meshes})...")

        with torch.no_grad():
            output = self.model(
                pts_t,
                normals=nrm_t,
                grasp_pts=grasp_pts_t,
                grasp_normals=grasp_nrm_t,
                n_grasps=n_grasps,
                each_object=True,
                return_meshes=return_meshes,
                return_scene_grasps=True,
                th=score_threshold,
            )

        _logger.info(
            f"Raw grasp tensors: rot={output.scene_grasp_poses[0].shape}, "
            f"centers={output.scene_grasp_poses[1].shape}, "
            f"scores={output.scene_grasp_poses[2].shape}, "
            f"class_preds={output.class_predictions.shape}"
        )

        if return_meshes:
            _logger.info(f"Reconstructed {len(output.reconstructions)} instance mesh(es).")
        _logger.info("Inference complete.")
        return output
