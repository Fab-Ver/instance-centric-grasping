import sys
import os
import torch
import numpy as np
from loguru import logger

class ICGNetPredictor:
    def __init__(self, config_path, icgnet_repo_path=None, device='cuda'):
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
                logger.info(f"Added {abs_repo_path} to sys.path")

        try:
            from icg_net import get_model
            logger.info(f"Loading model from: {config_path}")
            self.model = get_model(config_path, device=self.device)
            self.model.eval()
            logger.success(f"ICGNet loaded successfully on {self.device}")
        except ImportError as e:
            logger.error(f"Failed to import 'icg_net': {e}")
            logger.error("Make sure icg_net is cloned and icgnet_repo_path is correct.")
            raise
        except Exception as e:
            logger.error(f"Error loading model: {e}")
            raise

    def predict(self, points, normals, n_grasps=64):
        """
        Run inference on the given point cloud.

        :param points: numpy array (N, 3) of point positions.
        :param normals: numpy array (N, 3) of point normals.
        :param n_grasps: number of grasps to generate.
        :return: ModelPredOut with scene_grasp_poses and class_predictions.
        """
        from .pointcloud_utils import to_torch_tensors

        pts_t, nrm_t = to_torch_tensors(points, normals, device=self.device)

        logger.info(f"Running inference on {pts_t.shape[0]} points...")

        with torch.no_grad():
            # return_scene_grasps=True returns grasps in full scene space.
            output = self.model(
                pts_t,
                normals=nrm_t,
                grasp_pts=pts_t,
                grasp_normals=nrm_t,
                n_grasps=n_grasps,
                each_object=True,
                return_meshes=False,
                return_scene_grasps=True,
            )

        logger.success("Inference complete.")
        return output

    def get_grasps_as_poses(self, output):
        """Return raw scene_grasp_poses from ModelPredOut."""
        # Grasps are already in world frame — TF transform is done in grasp_service_node.
        return output.scene_grasp_poses
