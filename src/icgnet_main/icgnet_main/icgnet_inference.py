import sys
import os
import torch
import numpy as np
from loguru import logger

class ICGNetPredictor:
    def __init__(self, config_path, icgnet_repo_path=None, device='cuda'):
        """
        Inizializza il modello ICGNet.
        :param config_path: Percorso assoluto al file config.yaml del modello.
        :param icgnet_repo_path: Percorso alla cartella radice del repository icg_net.
        :param device: 'cuda' o 'cpu'.
        """
        self.device = device if torch.cuda.is_available() else 'cpu'
        
        # Aggiunta dinamica del repository icg_net al path di sistema
        if icgnet_repo_path:
            abs_repo_path = os.path.abspath(icgnet_repo_path)
            if abs_repo_path not in sys.path:
                sys.path.insert(0, abs_repo_path)
                logger.info(f"Aggiunto {abs_repo_path} al sys.path")
            
        try:
            # Importazione dei moduli dal repository icg_net
            from icg_net import get_model
            logger.info(f"Caricamento modello da: {config_path}")
            self.model = get_model(config_path, device=self.device)
            self.model.eval()
            logger.success(f"Modello ICGNet caricato correttamente su {self.device}")
        except ImportError as e:
            logger.error(f"Impossibile importare 'icg_net': {e}")
            logger.error("Assicurati di aver clonato il repository icg_net e di aver passato il path corretto.")
            raise
        except Exception as e:
            logger.error(f"Errore durante il caricamento del modello: {e}")
            raise

    def predict(self, points, normals, n_grasps=64):
        """
        Esegue l'inferenza sulla point cloud fornita.
        :param points: Array numpy (N, 3) delle posizioni dei punti.
        :param normals: Array numpy (N, 3) delle normali dei punti.
        :param n_grasps: Numero di grasp da generare.
        :return: Oggetto ModelPredOut contenente i risultati (grasps, labels, etc.)
        """
        from .pointcloud_utils import to_torch_tensors
        
        # Conversione in tensori PyTorch
        pts_t, nrm_t = to_torch_tensors(points, normals, device=self.device)
        
        logger.info(f"Esecuzione inferenza su {pts_t.shape[0]} punti...")
        
        with torch.no_grad():
            # L'interfaccia di ICGNet richiede la pointcloud e le normali.
            # return_scene_grasps=True restituisce i grasp nell'intero spazio della scena.
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
            
        logger.success("Inferenza completata.")
        return output

    def get_grasps_as_poses(self, output):
        """
        Converte l'output del modello in una lista di matrici di trasformazione 4x4.
        """
        # Questa logica dipende dalla struttura esatta di ModelPredOut (scene_grasp_poses)
        # Nel notebook out.scene_grasp_poses[0] contiene le posizioni e orientamenti.
        # Restituiamo una lista di matrici (pos, quat) o matrici 4x4.
        
        # NOTA: ICGNet restituisce i grasp nel frame della camera.
        # Sarà necessario trasformarli nel frame 'world' o 'base_link' nel nodo ROS.
        return output.scene_grasp_poses
