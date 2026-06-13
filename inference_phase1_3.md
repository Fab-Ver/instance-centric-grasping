alber@DESKTOP-L4POG0B:~/Robotics_Project/instance-centric-grasping$ ros2 launch icgnet_main icgnet_inference.launch.py
[INFO] [launch]: All log files can be found below /home/alber/.ros/log/2026-06-13-23-41-04-328793-DESKTOP-L4POG0B-136641
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [grasp_service_node-1]: process started with pid [136671]
[grasp_service_node-1] [INFO] [1781386869.138105375] [icgnet_predictor]: Added /home/alber/icg_net to sys.path
[grasp_service_node-1] [INFO] [1781386870.669615532] [icgnet_predictor]: Loading model from: /home/alber/Robotics_Project/instance-centric-grasping/icgnet_weights/config.yaml
[grasp_service_node-1] /home/alber/icg_net/icg_net/icg_net.py:77: UserWarning:
[grasp_service_node-1] The version_base parameter is not specified.
[grasp_service_node-1] Please specify a compatability version level, or None.
[grasp_service_node-1] Will assume defaults for version 1.1
[grasp_service_node-1]   with initialize_config_dir(config_dir=cfg_dir):
[grasp_service_node-1] 2026-06-13 23:41:12.922 | WARNING  | icg_net.utils.checkpoint:load_checkpoint_with_missing_or_exsessive_keys:51 - Key not found, it will be initialized randomly: criterion.empty_weight
[grasp_service_node-1] 2026-06-13 23:41:12.962 | WARNING  | icg_net.utils.checkpoint:load_checkpoint_with_missing_or_exsessive_keys:73 - excessive key: criterion.empty_weight
[grasp_service_node-1] [INFO] [1781386873.193287841] [icgnet_predictor]: ICGNet loaded successfully on cuda
[grasp_service_node-1] [INFO] [1781386873.194518723] [icgnet_grasp_node]: ICGNet loaded successfully.
[grasp_service_node-1] [INFO] [1781386873.312465504] [icgnet_grasp_node]: ICGNetGraspNode ready — topic=/camera/rgbd_camera/points, target_frame=world, n_grasps=32
[grasp_service_node-1] [INFO] [1781386910.573762346] [icgnet_grasp_node]: Starting grasp computation...
[grasp_service_node-1] [INFO] [1781386912.083444417] [icgnet_grasp_node]: Bin exclusion: removed 19017 points.
[grasp_service_node-1] [INFO] [1781386917.717237941] [icgnet_grasp_node]: Preprocessing: 307200 → seg=5719 (encoder), grasp=1988 (sampling) points
[grasp_service_node-1] [INFO] [1781386917.742878921] [icgnet_predictor]: Running inference on 5719 points (return_meshes=True)...
[grasp_service_node-1] /home/alber/icg_net/icg_net/utils/grasps.py:215: UserWarning: Using torch.cross without specifying the dim arg is deprecated.
[grasp_service_node-1] Please either pass the dim explicitly or simply use torch.linalg.cross.
[grasp_service_node-1] The default value of dim will change to agree with that of linalg.cross in a future release. (Triggered internally at ../aten/src/ATen/native/Cross.cpp:63.)
[grasp_service_node-1]   x_axis = torch.cross(gravity, y_axis)
[grasp_service_node-1] [INFO] [1781386925.356897898] [icgnet_predictor]: Raw grasp tensors: rot=torch.Size([7950, 3, 3]), centers=torch.Size([7950, 3]), scores=torch.Size([7950]), class_preds=torch.Size([4])
[grasp_service_node-1] [INFO] [1781386925.357747056] [icgnet_predictor]: Reconstructed 4 instance mesh(es).
[grasp_service_node-1] [INFO] [1781386925.358774734] [icgnet_predictor]: Inference complete.
[grasp_service_node-1] [INFO] [1781386925.374299522] [icgnet_grasp_node]: [RECON] inst_0 → class=box (id=1)
[grasp_service_node-1] [INFO] [1781386925.377071276] [icgnet_grasp_node]: [RECON] inst_1 → class=box (id=1)
[grasp_service_node-1] [INFO] [1781386925.378377939] [icgnet_grasp_node]: [RECON] inst_2 → class=ball (id=5)
[grasp_service_node-1] [INFO] [1781386925.379553526] [icgnet_grasp_node]: [RECON] inst_3 → class=ball (id=5)
[grasp_service_node-1] [INFO] [1781386926.746747756] [icgnet_grasp_node]: [RECON_VIZ] Published 4 reconstruction mesh marker(s) on /icgnet/reconstruction_meshes.
[grasp_service_node-1] [INFO] [1781386926.763935723] [icgnet_grasp_node]: [INSTANCES] 4 instance(s): inst_0=box(id=1, 5538g), inst_1=box(id=1, 1758g), inst_2=ball(id=5, 336g), inst_3=ball(id=5, 318g) | total_grasps=7950
[grasp_service_node-1] [INFO] [1781386926.768985011] [icgnet_grasp_node]: [GRASP_POS] inst_0 box (5538g): mean=(0.481,-0.286,0.092) x=[0.397,0.562] y=[-0.375,-0.203] z=[0.016,0.152]
[grasp_service_node-1] [INFO] [1781386926.771341654] [icgnet_grasp_node]: [GRASP_POS] inst_1 box (1758g): mean=(0.475,-0.295,0.083) x=[0.399,0.561] y=[-0.375,-0.207] z=[0.014,0.151]
[grasp_service_node-1] [INFO] [1781386926.773155265] [icgnet_grasp_node]: [GRASP_POS] inst_2 ball (336g): mean=(0.475,-0.299,0.076) x=[0.425,0.559] y=[-0.374,-0.245] z=[0.020,0.139]
[grasp_service_node-1] [INFO] [1781386926.774965616] [icgnet_grasp_node]: [GRASP_POS] inst_3 ball (318g): mean=(0.478,-0.302,0.078) x=[0.425,0.559] y=[-0.374,-0.245] z=[0.020,0.139]
[grasp_service_node-1] [INFO] [1781386926.779064757] [icgnet_grasp_node]: [SCORES] top-10: [0.6411, 0.636, 0.6336, 0.6226, 0.619, 0.6182, 0.618, 0.6164, 0.6162, 0.6132] | min=0.0000 max=0.6411 mean=0.2174 | >0.3: 3301 >0.5: 973 >0.7: 0
[grasp_service_node-1] [INFO] [1781386931.348804235] [icgnet_grasp_node]: [RECON_DIAG] 4 reconstruction(s) from ICGNet.
[grasp_service_node-1] [INFO] [1781386931.359172959] [icgnet_grasp_node]: [RECON_DIAG] inst=0: 5492v 10984f AABB=[0.443,-0.339,0.039]->[0.566,-0.239,0.114] centroid=(0.507,-0.283,0.079)
[grasp_service_node-1] [INFO] [1781386931.475025287] [icgnet_grasp_node]: Published CollisionObject 'icgnet_inst_0' (750 triangles, hull=True).
[grasp_service_node-1] [INFO] [1781386931.486753905] [icgnet_grasp_node]: [RECON_DIAG] inst=1: 6132v 12284f AABB=[0.440,-0.335,0.041]->[0.564,-0.239,0.114] centroid=(0.509,-0.281,0.079)
[grasp_service_node-1] [INFO] [1781386931.580140698] [icgnet_grasp_node]: Published CollisionObject 'icgnet_inst_1' (676 triangles, hull=True).
[grasp_service_node-1] [INFO] [1781386931.590805871] [icgnet_grasp_node]: [RECON_DIAG] inst=2: 4619v 9164f AABB=[0.441,-0.331,0.040]->[0.526,-0.239,0.111] centroid=(0.480,-0.281,0.079)
[grasp_service_node-1] [INFO] [1781386931.658018694] [icgnet_grasp_node]: Published CollisionObject 'icgnet_inst_2' (332 triangles, hull=True).
[grasp_service_node-1] [INFO] [1781386931.667823452] [icgnet_grasp_node]: [RECON_DIAG] inst=3: 5122v 10244f AABB=[0.443,-0.332,0.040]->[0.563,-0.240,0.113] centroid=(0.510,-0.281,0.078)
[grasp_service_node-1] [INFO] [1781386931.747365888] [icgnet_grasp_node]: Published CollisionObject 'icgnet_inst_3' (656 triangles, hull=True).
[grasp_service_node-1] [WARN] [1781386931.749288380] [icgnet_grasp_node]: [RECON_DIAG] AABB OVERLAP: inst 0 ↔ inst 1. Possible Mask3D under-segmentation or convex-hull inflation.
[grasp_service_node-1] [WARN] [1781386931.750055693] [icgnet_grasp_node]: [RECON_DIAG] AABB OVERLAP: inst 0 ↔ inst 2. Possible Mask3D under-segmentation or convex-hull inflation.
[grasp_service_node-1] [WARN] [1781386931.751050024] [icgnet_grasp_node]: [RECON_DIAG] AABB OVERLAP: inst 0 ↔ inst 3. Possible Mask3D under-segmentation or convex-hull inflation.
[grasp_service_node-1] [WARN] [1781386931.752346377] [icgnet_grasp_node]: [RECON_DIAG] AABB OVERLAP: inst 1 ↔ inst 2. Possible Mask3D under-segmentation or convex-hull inflation.
[grasp_service_node-1] [WARN] [1781386931.753776858] [icgnet_grasp_node]: [RECON_DIAG] AABB OVERLAP: inst 1 ↔ inst 3. Possible Mask3D under-segmentation or convex-hull inflation.
[grasp_service_node-1] [WARN] [1781386931.756091317] [icgnet_grasp_node]: [RECON_DIAG] AABB OVERLAP: inst 2 ↔ inst 3. Possible Mask3D under-segmentation or convex-hull inflation.
[grasp_service_node-1] [INFO] [1781386931.758760082] [icgnet_grasp_node]: Published 4 collision object(s) to MoveIt2.
[grasp_service_node-1] [INFO] [1781386931.778169723] [icgnet_grasp_node]: Published 7950 grasps
[grasp_service_node-1] [INFO] [1781386945.769912276] [icgnet_grasp_node]: Starting grasp computation...
[grasp_service_node-1] [INFO] [1781386947.465924864] [icgnet_grasp_node]: Bin exclusion: removed 20327 points.
[grasp_service_node-1] [INFO] [1781386947.594749417] [icgnet_grasp_node]: Preprocessing: 307200 → seg=9026 (encoder), grasp=2702 (sampling) points
[grasp_service_node-1] [INFO] [1781386947.624965899] [icgnet_predictor]: Running inference on 9026 points (return_meshes=True)...
[grasp_service_node-1] [INFO] [1781386950.624732548] [icgnet_predictor]: Raw grasp tensors: rot=torch.Size([8106, 3, 3]), centers=torch.Size([8106, 3]), scores=torch.Size([8106]), class_preds=torch.Size([3])
[grasp_service_node-1] [INFO] [1781386950.626029017] [icgnet_predictor]: Reconstructed 3 instance mesh(es).
[grasp_service_node-1] [INFO] [1781386950.627911448] [icgnet_predictor]: Inference complete.
[grasp_service_node-1] [INFO] [1781386950.638071188] [icgnet_grasp_node]: [RECON] inst_0 → class=box (id=1)
[grasp_service_node-1] [INFO] [1781386950.639124166] [icgnet_grasp_node]: [RECON] inst_1 → class=ball (id=5)
[grasp_service_node-1] [INFO] [1781386950.640090629] [icgnet_grasp_node]: [RECON] inst_2 → class=ball (id=5)
[grasp_service_node-1] [INFO] [1781386951.762402313] [icgnet_grasp_node]: [RECON_VIZ] Published 3 reconstruction mesh marker(s) on /icgnet/reconstruction_meshes.
[grasp_service_node-1] [INFO] [1781386951.777783950] [icgnet_grasp_node]: [INSTANCES] 3 instance(s): inst_0=box(id=1, 6096g), inst_1=ball(id=5, 306g), inst_2=ball(id=5, 1704g) | total_grasps=8106
[grasp_service_node-1] [INFO] [1781386951.780781685] [icgnet_grasp_node]: [GRASP_POS] inst_0 box (6096g): mean=(0.634,-0.268,0.096) x=[0.574,0.788] y=[-0.339,-0.165] z=[0.009,0.150]
[grasp_service_node-1] [INFO] [1781386951.782004314] [icgnet_grasp_node]: [GRASP_POS] inst_1 ball (306g): mean=(0.649,-0.256,0.100) x=[0.576,0.760] y=[-0.334,-0.204] z=[0.017,0.141]
[grasp_service_node-1] [INFO] [1781386951.783227789] [icgnet_grasp_node]: [GRASP_POS] inst_2 ball (1704g): mean=(0.634,-0.271,0.098) x=[0.576,0.771] y=[-0.338,-0.178] z=[0.009,0.148]
[grasp_service_node-1] [INFO] [1781386951.787083075] [icgnet_grasp_node]: [SCORES] top-10: [0.5854, 0.5812, 0.58, 0.5728, 0.5723, 0.5712, 0.5705, 0.5697, 0.5683, 0.568] | min=0.0000 max=0.5854 mean=0.2033 | >0.3: 2908 >0.5: 318 >0.7: 0
[grasp_service_node-1] [INFO] [1781386954.575620188] [icgnet_grasp_node]: [RECON_DIAG] 3 reconstruction(s) from ICGNet.
[grasp_service_node-1] [INFO] [1781386954.586659558] [icgnet_grasp_node]: [RECON_DIAG] inst=0: 6090v 12176f AABB=[0.604,-0.314,0.040]->[0.718,-0.200,0.113] centroid=(0.659,-0.256,0.079)
[grasp_service_node-1] [INFO] [1781386954.623802921] [icgnet_grasp_node]: Published CollisionObject 'icgnet_inst_0' (912 triangles, hull=True).
[grasp_service_node-1] [INFO] [1781386954.636141083] [icgnet_grasp_node]: [RECON_DIAG] inst=1: 6558v 13116f AABB=[0.607,-0.306,0.003]->[0.737,-0.200,0.113] centroid=(0.664,-0.252,0.078)
[grasp_service_node-1] [INFO] [1781386954.668272976] [icgnet_grasp_node]: Published CollisionObject 'icgnet_inst_1' (748 triangles, hull=True).
[grasp_service_node-1] [INFO] [1781386954.678760187] [icgnet_grasp_node]: [RECON_DIAG] inst=2: 5854v 11704f AABB=[0.606,-0.308,0.003]->[0.738,-0.200,0.113] centroid=(0.664,-0.257,0.078)
[grasp_service_node-1] [INFO] [1781386954.709714556] [icgnet_grasp_node]: Published CollisionObject 'icgnet_inst_2' (740 triangles, hull=True).
[grasp_service_node-1] [WARN] [1781386954.711023068] [icgnet_grasp_node]: [RECON_DIAG] AABB OVERLAP: inst 0 ↔ inst 1. Possible Mask3D under-segmentation or convex-hull inflation.
[grasp_service_node-1] [WARN] [1781386954.712169428] [icgnet_grasp_node]: [RECON_DIAG] AABB OVERLAP: inst 0 ↔ inst 2. Possible Mask3D under-segmentation or convex-hull inflation.
[grasp_service_node-1] [WARN] [1781386954.713268886] [icgnet_grasp_node]: [RECON_DIAG] AABB OVERLAP: inst 1 ↔ inst 2. Possible Mask3D under-segmentation or convex-hull inflation.
[grasp_service_node-1] [INFO] [1781386954.717517560] [icgnet_grasp_node]: Published 3 collision object(s) to MoveIt2.
[grasp_service_node-1] [INFO] [1781386954.736686032] [icgnet_grasp_node]: Published 8106 grasps
[grasp_service_node-1] [INFO] [1781386970.985421837] [icgnet_grasp_node]: Starting grasp computation...
[grasp_service_node-1] [INFO] [1781386973.035440647] [icgnet_grasp_node]: Bin exclusion: removed 21880 points.
[grasp_service_node-1] [INFO] [1781386973.171416609] [icgnet_grasp_node]: Preprocessing: 307200 → seg=6803 (encoder), grasp=2358 (sampling) points
[grasp_service_node-1] [INFO] [1781386973.208385599] [icgnet_predictor]: Running inference on 6803 points (return_meshes=True)...
[grasp_service_node-1] [INFO] [1781386977.101520809] [icgnet_predictor]: Raw grasp tensors: rot=torch.Size([9432, 3, 3]), centers=torch.Size([9432, 3]), scores=torch.Size([9432]), class_preds=torch.Size([4])
[grasp_service_node-1] [INFO] [1781386977.102683968] [icgnet_predictor]: Reconstructed 4 instance mesh(es).
[grasp_service_node-1] [INFO] [1781386977.103664807] [icgnet_predictor]: Inference complete.
[grasp_service_node-1] [INFO] [1781386977.117968977] [icgnet_grasp_node]: [RECON] inst_0 → class=box (id=1)
[grasp_service_node-1] [INFO] [1781386977.119407575] [icgnet_grasp_node]: [RECON] inst_1 → class=ball (id=5)
[grasp_service_node-1] [INFO] [1781386977.120318655] [icgnet_grasp_node]: [RECON] inst_2 → class=ball (id=5)
[grasp_service_node-1] [INFO] [1781386977.121198978] [icgnet_grasp_node]: [RECON] inst_3 → class=ball (id=5)
[grasp_service_node-1] [INFO] [1781386978.127713244] [icgnet_grasp_node]: [RECON_VIZ] Published 4 reconstruction mesh marker(s) on /icgnet/reconstruction_meshes.
[grasp_service_node-1] [INFO] [1781386978.144906342] [icgnet_grasp_node]: [INSTANCES] 4 instance(s): inst_0=box(id=1, 6432g), inst_1=ball(id=5, 24g), inst_2=ball(id=5, 270g), inst_3=ball(id=5, 2706g) | total_grasps=9432
[grasp_service_node-1] [INFO] [1781386978.147965965] [icgnet_grasp_node]: [GRASP_POS] inst_0 box (6432g): mean=(0.517,-0.187,0.097) x=[0.458,0.598] y=[-0.322,-0.101] z=[0.007,0.150]
[grasp_service_node-1] [INFO] [1781386978.149943073] [icgnet_grasp_node]: [GRASP_POS] inst_1 ball (24g): mean=(0.514,-0.284,0.023) x=[0.496,0.540] y=[-0.305,-0.257] z=[0.017,0.029]
[grasp_service_node-1] [INFO] [1781386978.153911345] [icgnet_grasp_node]: [GRASP_POS] inst_2 ball (270g): mean=(0.498,-0.170,0.102) x=[0.460,0.561] y=[-0.308,-0.103] z=[0.017,0.149]
[grasp_service_node-1] [INFO] [1781386978.156092970] [icgnet_grasp_node]: [GRASP_POS] inst_3 ball (2706g): mean=(0.520,-0.175,0.098) x=[0.461,0.596] y=[-0.308,-0.106] z=[0.017,0.150]
[grasp_service_node-1] [INFO] [1781386978.158354922] [icgnet_grasp_node]: [SCORES] top-10: [0.5919, 0.5885, 0.5863, 0.5731, 0.5683, 0.5584, 0.5582, 0.5569, 0.5563, 0.5551] | min=0.0000 max=0.5919 mean=0.2231 | >0.3: 4063 >0.5: 224 >0.7: 0
[grasp_service_node-1] [INFO] [1781386984.609154745] [icgnet_grasp_node]: [RECON_DIAG] 4 reconstruction(s) from ICGNet.
[grasp_service_node-1] [INFO] [1781386984.625962054] [icgnet_grasp_node]: [RECON_DIAG] inst=0: 6430v 12864f AABB=[0.494,-0.262,0.040]->[0.599,-0.139,0.113] centroid=(0.544,-0.192,0.080)
[grasp_service_node-1] [INFO] [1781386984.670945586] [icgnet_grasp_node]: Published CollisionObject 'icgnet_inst_0' (888 triangles, hull=True).
[grasp_service_node-1] [INFO] [1781386984.674075221] [icgnet_grasp_node]: [RECON_DIAG] inst=1: 878v 1752f AABB=[0.533,-0.267,0.041]->[0.554,-0.239,0.088] centroid=(0.544,-0.254,0.070)
[grasp_service_node-1] [INFO] [1781386984.696573908] [icgnet_grasp_node]: Published CollisionObject 'icgnet_inst_1' (304 triangles, hull=True).
[grasp_service_node-1] [INFO] [1781386984.701130562] [icgnet_grasp_node]: [RECON_DIAG] inst=2: 1130v 2244f AABB=[0.518,-0.267,0.041]->[0.557,-0.233,0.110] centroid=(0.543,-0.251,0.074)
[grasp_service_node-1] [INFO] [1781386984.719759370] [icgnet_grasp_node]: Published CollisionObject 'icgnet_inst_2' (236 triangles, hull=True).
[grasp_service_node-1] [INFO] [1781386984.730349785] [icgnet_grasp_node]: [RECON_DIAG] inst=3: 5922v 11848f AABB=[0.495,-0.267,0.039]->[0.598,-0.140,0.113] centroid=(0.545,-0.195,0.079)
[grasp_service_node-1] [INFO] [1781386984.772607145] [icgnet_grasp_node]: Published CollisionObject 'icgnet_inst_3' (852 triangles, hull=True).
[grasp_service_node-1] [WARN] [1781386984.774295918] [icgnet_grasp_node]: [RECON_DIAG] AABB OVERLAP: inst 0 ↔ inst 1. Possible Mask3D under-segmentation or convex-hull inflation.
[grasp_service_node-1] [WARN] [1781386984.775871197] [icgnet_grasp_node]: [RECON_DIAG] AABB OVERLAP: inst 0 ↔ inst 2. Possible Mask3D under-segmentation or convex-hull inflation.
[grasp_service_node-1] [WARN] [1781386984.777453661] [icgnet_grasp_node]: [RECON_DIAG] AABB OVERLAP: inst 0 ↔ inst 3. Possible Mask3D under-segmentation or convex-hull inflation.
[grasp_service_node-1] [WARN] [1781386984.779567560] [icgnet_grasp_node]: [RECON_DIAG] AABB OVERLAP: inst 1 ↔ inst 2. Possible Mask3D under-segmentation or convex-hull inflation.
[grasp_service_node-1] [WARN] [1781386984.780853715] [icgnet_grasp_node]: [RECON_DIAG] AABB OVERLAP: inst 1 ↔ inst 3. Possible Mask3D under-segmentation or convex-hull inflation.
[grasp_service_node-1] [WARN] [1781386984.782248504] [icgnet_grasp_node]: [RECON_DIAG] AABB OVERLAP: inst 2 ↔ inst 3. Possible Mask3D under-segmentation or convex-hull inflation.
[grasp_service_node-1] [INFO] [1781386984.784222913] [icgnet_grasp_node]: Published 4 collision object(s) to MoveIt2.
[grasp_service_node-1] [INFO] [1781386984.813524773] [icgnet_grasp_node]: Published 9432 grasps
[grasp_service_node-1] [INFO] [1781386998.476822936] [icgnet_grasp_node]: Starting grasp computation...
[grasp_service_node-1] [INFO] [1781387000.056346683] [icgnet_grasp_node]: Bin exclusion: removed 21880 points.
[grasp_service_node-1] [INFO] [1781387000.162962259] [icgnet_grasp_node]: Preprocessing: 307200 → seg=5457 (encoder), grasp=1381 (sampling) points
[grasp_service_node-1] [INFO] [1781387000.184283979] [icgnet_predictor]: Running inference on 5457 points (return_meshes=True)...
[grasp_service_node-1] [INFO] [1781387001.358316768] [icgnet_predictor]: Raw grasp tensors: rot=torch.Size([1380, 3, 3]), centers=torch.Size([1380, 3]), scores=torch.Size([1380]), class_preds=torch.Size([1])
[grasp_service_node-1] [INFO] [1781387001.359173839] [icgnet_predictor]: Reconstructed 1 instance mesh(es).
[grasp_service_node-1] [INFO] [1781387001.359951957] [icgnet_predictor]: Inference complete.
[grasp_service_node-1] [INFO] [1781387001.364504190] [icgnet_grasp_node]: [RECON] inst_0 → class=box (id=1)
[grasp_service_node-1] [INFO] [1781387001.557914896] [icgnet_grasp_node]: [RECON_VIZ] Published 1 reconstruction mesh marker(s) on /icgnet/reconstruction_meshes.
[grasp_service_node-1] [INFO] [1781387001.562171821] [icgnet_grasp_node]: [INSTANCES] 1 instance(s): inst_0=box(id=1, 1380g) | total_grasps=1380
[grasp_service_node-1] [INFO] [1781387001.564759205] [icgnet_grasp_node]: [GRASP_POS] inst_0 box (1380g): mean=(0.669,0.051,0.057) x=[0.621,0.708] y=[-0.018,0.109] z=[0.012,0.117]
[grasp_service_node-1] [INFO] [1781387001.567058892] [icgnet_grasp_node]: [SCORES] top-10: [0.3423, 0.3315, 0.3247, 0.317, 0.3117, 0.305, 0.3034, 0.2993, 0.2992, 0.2914] | min=0.0000 max=0.3423 mean=0.0191 | >0.3: 7 >0.5: 0 >0.7: 0
[grasp_service_node-1] [INFO] [1781387002.315419439] [icgnet_grasp_node]: [RECON_DIAG] 1 reconstruction(s) from ICGNet.
[grasp_service_node-1] [INFO] [1781387002.323072306] [icgnet_grasp_node]: [RECON_DIAG] inst=0: 3613v 7208f AABB=[0.616,-0.017,-0.039]->[0.717,0.096,0.087] centroid=(0.665,0.042,0.064)
[grasp_service_node-1] [INFO] [1781387002.351460397] [icgnet_grasp_node]: Published CollisionObject 'icgnet_inst_0' (598 triangles, hull=True).
[grasp_service_node-1] [INFO] [1781387002.353327267] [icgnet_grasp_node]: Published 1 collision object(s) to MoveIt2.
[grasp_service_node-1] [INFO] [1781387002.358021553] [icgnet_grasp_node]: Published 1380 grasps
[grasp_service_node-1] [INFO] [1781387077.532322540] [icgnet_grasp_node]: Starting grasp computation...
[grasp_service_node-1] [INFO] [1781387079.122380297] [icgnet_grasp_node]: Bin exclusion: removed 21880 points.
[grasp_service_node-1] [INFO] [1781387079.224784129] [icgnet_grasp_node]: Preprocessing: 307200 → seg=5248 (encoder), grasp=1368 (sampling) points
[grasp_service_node-1] [INFO] [1781387079.242173764] [icgnet_predictor]: Running inference on 5248 points (return_meshes=True)...
[grasp_service_node-1] [INFO] [1781387080.471134646] [icgnet_predictor]: Raw grasp tensors: rot=torch.Size([1368, 3, 3]), centers=torch.Size([1368, 3]), scores=torch.Size([1368]), class_preds=torch.Size([1])
[grasp_service_node-1] [INFO] [1781387080.472452940] [icgnet_predictor]: Reconstructed 1 instance mesh(es).
[grasp_service_node-1] [INFO] [1781387080.473500904] [icgnet_predictor]: Inference complete.
[grasp_service_node-1] [INFO] [1781387080.479263601] [icgnet_grasp_node]: [RECON] inst_0 → class=box (id=1)
[grasp_service_node-1] [INFO] [1781387081.013129001] [icgnet_grasp_node]: [RECON_VIZ] Published 1 reconstruction mesh marker(s) on /icgnet/reconstruction_meshes.
[grasp_service_node-1] [INFO] [1781387081.021023508] [icgnet_grasp_node]: [INSTANCES] 1 instance(s): inst_0=box(id=1, 1368g) | total_grasps=1368
[grasp_service_node-1] [INFO] [1781387081.022663905] [icgnet_grasp_node]: [GRASP_POS] inst_0 box (1368g): mean=(0.592,0.228,0.042) x=[0.544,0.635] y=[0.161,0.288] z=[0.010,0.103]
[grasp_service_node-1] [INFO] [1781387081.023798997] [icgnet_grasp_node]: [SCORES] top-10: [0.1189, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001] | min=0.0000 max=0.1189 mean=0.0001 | >0.3: 0 >0.5: 0 >0.7: 0
[grasp_service_node-1] [INFO] [1781387081.746074407] [icgnet_grasp_node]: [RECON_DIAG] 1 reconstruction(s) from ICGNet.
[grasp_service_node-1] [INFO] [1781387081.762491001] [icgnet_grasp_node]: [RECON_DIAG] inst=0: 6184v 12330f AABB=[0.537,0.169,-0.041]->[0.641,0.271,0.086] centroid=(0.592,0.222,0.043)
[grasp_service_node-1] [INFO] [1781387081.796165800] [icgnet_grasp_node]: Published CollisionObject 'icgnet_inst_0' (686 triangles, hull=True).
[grasp_service_node-1] [INFO] [1781387081.798363378] [icgnet_grasp_node]: Published 1 collision object(s) to MoveIt2.
[grasp_service_node-1] [INFO] [1781387081.804724506] [icgnet_grasp_node]: Published 1368 grasps
[grasp_service_node-1] [INFO] [1781387218.497929635] [icgnet_grasp_node]: Starting grasp computation...
[grasp_service_node-1] [INFO] [1781387220.076830016] [icgnet_grasp_node]: Bin exclusion: removed 21880 points.
[grasp_service_node-1] [INFO] [1781387220.175205081] [icgnet_grasp_node]: Preprocessing: 307200 → seg=5255 (encoder), grasp=1411 (sampling) points
[grasp_service_node-1] [INFO] [1781387220.202098166] [icgnet_predictor]: Running inference on 5255 points (return_meshes=True)...
[grasp_service_node-1] [INFO] [1781387221.586956148] [icgnet_predictor]: Raw grasp tensors: rot=torch.Size([1410, 3, 3]), centers=torch.Size([1410, 3]), scores=torch.Size([1410]), class_preds=torch.Size([1])
[grasp_service_node-1] [INFO] [1781387221.587739241] [icgnet_predictor]: Reconstructed 1 instance mesh(es).
[grasp_service_node-1] [INFO] [1781387221.588471126] [icgnet_predictor]: Inference complete.
[grasp_service_node-1] [INFO] [1781387221.593580829] [icgnet_grasp_node]: [RECON] inst_0 → class=box (id=1)
[grasp_service_node-1] [INFO] [1781387221.794292954] [icgnet_grasp_node]: [RECON_VIZ] Published 1 reconstruction mesh marker(s) on /icgnet/reconstruction_meshes.
[grasp_service_node-1] [INFO] [1781387221.803765602] [icgnet_grasp_node]: [INSTANCES] 1 instance(s): inst_0=box(id=1, 1410g) | total_grasps=1410
[grasp_service_node-1] [INFO] [1781387221.806414051] [icgnet_grasp_node]: [GRASP_POS] inst_0 box (1410g): mean=(0.621,0.126,0.060) x=[0.565,0.654] y=[0.046,0.177] z=[0.012,0.121]
[grasp_service_node-1] [INFO] [1781387221.808199727] [icgnet_grasp_node]: [SCORES] top-10: [0.3215, 0.32, 0.3149, 0.3147, 0.3122, 0.3108, 0.2998, 0.2936, 0.2878, 0.2875] | min=0.0000 max=0.3215 mean=0.0260 | >0.3: 6 >0.5: 0 >0.7: 0
[grasp_service_node-1] [INFO] [1781387222.571523140] [icgnet_grasp_node]: [RECON_DIAG] 1 reconstruction(s) from ICGNet.
[grasp_service_node-1] [INFO] [1781387222.581116688] [icgnet_grasp_node]: [RECON_DIAG] inst=0: 3630v 7256f AABB=[0.560,0.048,0.036]->[0.661,0.159,0.087] centroid=(0.610,0.110,0.066)
[grasp_service_node-1] [INFO] [1781387222.618214730] [icgnet_grasp_node]: Published CollisionObject 'icgnet_inst_0' (758 triangles, hull=True).
[grasp_service_node-1] [INFO] [1781387222.620273124] [icgnet_grasp_node]: Published 1 collision object(s) to MoveIt2.
[grasp_service_node-1] [INFO] [1781387222.626676401] [icgnet_grasp_node]: Published 1410 grasps
[grasp_service_node-1] [INFO] [1781387266.899749916] [icgnet_grasp_node]: Starting grasp computation...
[grasp_service_node-1] [INFO] [1781387268.836045367] [icgnet_grasp_node]: Bin exclusion: removed 21880 points.
[grasp_service_node-1] [INFO] [1781387268.940402134] [icgnet_grasp_node]: Preprocessing: 307200 → seg=4290 (encoder), grasp=1236 (sampling) points
[grasp_service_node-1] [INFO] [1781387268.958579683] [icgnet_predictor]: Running inference on 4290 points (return_meshes=True)...
[grasp_service_node-1] [INFO] [1781387270.635727421] [icgnet_predictor]: Raw grasp tensors: rot=torch.Size([1236, 3, 3]), centers=torch.Size([1236, 3]), scores=torch.Size([1236]), class_preds=torch.Size([1])
[grasp_service_node-1] [INFO] [1781387270.636851372] [icgnet_predictor]: Reconstructed 1 instance mesh(es).
[grasp_service_node-1] [INFO] [1781387270.638236444] [icgnet_predictor]: Inference complete.
[grasp_service_node-1] [INFO] [1781387270.645428419] [icgnet_grasp_node]: [RECON] inst_0 → class=can (id=2)
[grasp_service_node-1] [INFO] [1781387270.937658710] [icgnet_grasp_node]: [RECON_VIZ] Published 1 reconstruction mesh marker(s) on /icgnet/reconstruction_meshes.
[grasp_service_node-1] [INFO] [1781387270.946728305] [icgnet_grasp_node]: [INSTANCES] 1 instance(s): inst_0=can(id=2, 1236g) | total_grasps=1236
[grasp_service_node-1] [INFO] [1781387270.948472352] [icgnet_grasp_node]: [GRASP_POS] inst_0 can (1236g): mean=(0.604,0.099,0.071) x=[0.559,0.645] y=[0.055,0.149] z=[0.013,0.143]
[grasp_service_node-1] [INFO] [1781387270.950269659] [icgnet_grasp_node]: [SCORES] top-10: [0.651, 0.645, 0.644, 0.6432, 0.6429, 0.6398, 0.6391, 0.6385, 0.6375, 0.636] | min=0.0000 max=0.6510 mean=0.1458 | >0.3: 320 >0.5: 84 >0.7: 0
[grasp_service_node-1] [INFO] [1781387271.854466227] [icgnet_grasp_node]: [RECON_DIAG] 1 reconstruction(s) from ICGNet.
[grasp_service_node-1] [INFO] [1781387271.866133066] [icgnet_grasp_node]: [RECON_DIAG] inst=0: 3998v 7972f AABB=[0.566,0.062,0.048]->[0.635,0.131,0.123] centroid=(0.602,0.098,0.085)
[grasp_service_node-1] [INFO] [1781387271.921536546] [icgnet_grasp_node]: Published CollisionObject 'icgnet_inst_0' (1124 triangles, hull=True).
[grasp_service_node-1] [INFO] [1781387271.923292120] [icgnet_grasp_node]: Published 1 collision object(s) to MoveIt2.
[grasp_service_node-1] [INFO] [1781387271.928562014] [icgnet_grasp_node]: Published 1236 grasps
[grasp_service_node-1] [INFO] [1781387316.928512955] [icgnet_grasp_node]: Starting grasp computation...
[grasp_service_node-1] [INFO] [1781387318.701307049] [icgnet_grasp_node]: Bin exclusion: removed 21880 points.
[grasp_service_node-1] [INFO] [1781387318.782571111] [icgnet_grasp_node]: Preprocessing: 307200 → seg=3913 (encoder), grasp=1234 (sampling) points
[grasp_service_node-1] [INFO] [1781387318.805511027] [icgnet_predictor]: Running inference on 3913 points (return_meshes=True)...
[grasp_service_node-1] [INFO] [1781387320.208193325] [icgnet_predictor]: Raw grasp tensors: rot=torch.Size([1230, 3, 3]), centers=torch.Size([1230, 3]), scores=torch.Size([1230]), class_preds=torch.Size([1])
[grasp_service_node-1] [INFO] [1781387320.209070138] [icgnet_predictor]: Reconstructed 1 instance mesh(es).
[grasp_service_node-1] [INFO] [1781387320.210142129] [icgnet_predictor]: Inference complete.
[grasp_service_node-1] [INFO] [1781387320.215191870] [icgnet_grasp_node]: [RECON] inst_0 → class=can (id=2)
[grasp_service_node-1] [INFO] [1781387320.445648840] [icgnet_grasp_node]: [RECON_VIZ] Published 1 reconstruction mesh marker(s) on /icgnet/reconstruction_meshes.
[grasp_service_node-1] [INFO] [1781387320.455195992] [icgnet_grasp_node]: [INSTANCES] 1 instance(s): inst_0=can(id=2, 1230g) | total_grasps=1230
[grasp_service_node-1] [INFO] [1781387320.456881107] [icgnet_grasp_node]: [GRASP_POS] inst_0 can (1230g): mean=(0.507,-0.087,0.107) x=[0.471,0.543] y=[-0.145,-0.054] z=[0.012,0.144]
[grasp_service_node-1] [INFO] [1781387320.458422853] [icgnet_grasp_node]: [SCORES] top-10: [0.7731, 0.7718, 0.7706, 0.7668, 0.7627, 0.7618, 0.7608, 0.7585, 0.7574, 0.7565] | min=0.0000 max=0.7731 mean=0.4094 | >0.3: 799 >0.5: 755 >0.7: 105
[grasp_service_node-1] [INFO] [1781387321.110735107] [icgnet_grasp_node]: [RECON_DIAG] 1 reconstruction(s) from ICGNet.
[grasp_service_node-1] [INFO] [1781387321.122117404] [icgnet_grasp_node]: [RECON_DIAG] inst=0: 4634v 9248f AABB=[0.472,-0.135,-0.020]->[0.541,-0.066,0.123] centroid=(0.510,-0.100,0.071)
[grasp_service_node-1] [INFO] [1781387321.164825309] [icgnet_grasp_node]: Published CollisionObject 'icgnet_inst_0' (1102 triangles, hull=True).
[grasp_service_node-1] [INFO] [1781387321.166441806] [icgnet_grasp_node]: Published 1 collision object(s) to MoveIt2.
[grasp_service_node-1] [INFO] [1781387321.171761686] [icgnet_grasp_node]: Published 1230 grasps
[grasp_service_node-1] [INFO] [1781387377.146910899] [icgnet_grasp_node]: Starting grasp computation...
[grasp_service_node-1] [INFO] [1781387378.606416791] [icgnet_grasp_node]: Bin exclusion: removed 21496 points.
[grasp_service_node-1] [INFO] [1781387378.728202054] [icgnet_grasp_node]: Preprocessing: 307200 → seg=4713 (encoder), grasp=1416 (sampling) points
[grasp_service_node-1] [INFO] [1781387378.744789526] [icgnet_predictor]: Running inference on 4713 points (return_meshes=True)...
[grasp_service_node-1] [INFO] [1781387380.190378500] [icgnet_predictor]: Raw grasp tensors: rot=torch.Size([1416, 3, 3]), centers=torch.Size([1416, 3]), scores=torch.Size([1416]), class_preds=torch.Size([1])
[grasp_service_node-1] [INFO] [1781387380.191554939] [icgnet_predictor]: Reconstructed 1 instance mesh(es).
[grasp_service_node-1] [INFO] [1781387380.192541812] [icgnet_predictor]: Inference complete.
[grasp_service_node-1] [INFO] [1781387380.198653671] [icgnet_grasp_node]: [RECON] inst_0 → class=can (id=2)
[grasp_service_node-1] [INFO] [1781387380.449963563] [icgnet_grasp_node]: [RECON_VIZ] Published 1 reconstruction mesh marker(s) on /icgnet/reconstruction_meshes.
[grasp_service_node-1] [INFO] [1781387380.458128294] [icgnet_grasp_node]: [INSTANCES] 1 instance(s): inst_0=can(id=2, 1416g) | total_grasps=1416
[grasp_service_node-1] [INFO] [1781387380.459865819] [icgnet_grasp_node]: [GRASP_POS] inst_0 can (1416g): mean=(0.574,-0.237,0.086) x=[0.540,0.610] y=[-0.292,-0.197] z=[0.011,0.144]
[grasp_service_node-1] [INFO] [1781387380.464205349] [icgnet_grasp_node]: [SCORES] top-10: [0.7791, 0.7782, 0.7777, 0.7741, 0.7726, 0.7707, 0.7701, 0.7658, 0.7649, 0.762] | min=0.0000 max=0.7791 mean=0.3130 | >0.3: 685 >0.5: 629 >0.7: 172
[grasp_service_node-1] [INFO] [1781387381.235783844] [icgnet_grasp_node]: [RECON_DIAG] 1 reconstruction(s) from ICGNet.
[grasp_service_node-1] [INFO] [1781387381.248457679] [icgnet_grasp_node]: [RECON_DIAG] inst=0: 4572v 9128f AABB=[0.545,-0.280,-0.017]->[0.610,-0.212,0.123] centroid=(0.582,-0.244,0.069)
[grasp_service_node-1] [INFO] [1781387381.292189057] [icgnet_grasp_node]: Published CollisionObject 'icgnet_inst_0' (1186 triangles, hull=True).
[grasp_service_node-1] [INFO] [1781387381.296113591] [icgnet_grasp_node]: Published 1 collision object(s) to MoveIt2.
[grasp_service_node-1] [INFO] [1781387381.303594207] [icgnet_grasp_node]: Published 1416 grasps
[grasp_service_node-1] [INFO] [1781387570.133101985] [icgnet_grasp_node]: Starting grasp computation...
[grasp_service_node-1] [INFO] [1781387571.662079748] [icgnet_grasp_node]: Bin exclusion: removed 21880 points.
[grasp_service_node-1] [INFO] [1781387571.773110315] [icgnet_grasp_node]: Preprocessing: 307200 → seg=6346 (encoder), grasp=1671 (sampling) points
[grasp_service_node-1] [INFO] [1781387571.795621893] [icgnet_predictor]: Running inference on 6346 points (return_meshes=True)...
[grasp_service_node-1] [INFO] [1781387575.034338265] [icgnet_predictor]: Raw grasp tensors: rot=torch.Size([6684, 3, 3]), centers=torch.Size([6684, 3]), scores=torch.Size([6684]), class_preds=torch.Size([4])
[grasp_service_node-1] [INFO] [1781387575.035099283] [icgnet_predictor]: Reconstructed 4 instance mesh(es).
[grasp_service_node-1] [INFO] [1781387575.035912190] [icgnet_predictor]: Inference complete.
[grasp_service_node-1] [INFO] [1781387575.047250200] [icgnet_grasp_node]: [RECON] inst_0 → class=ball (id=5)
[grasp_service_node-1] [INFO] [1781387575.048573975] [icgnet_grasp_node]: [RECON] inst_1 → class=cylindric (id=4)
[grasp_service_node-1] [INFO] [1781387575.049282891] [icgnet_grasp_node]: [RECON] inst_2 → class=bottle (id=3)
[grasp_service_node-1] [INFO] [1781387575.052034222] [icgnet_grasp_node]: [RECON] inst_3 → class=cylindric (id=4)
[grasp_service_node-1] [INFO] [1781387575.900400848] [icgnet_grasp_node]: [RECON_VIZ] Published 4 reconstruction mesh marker(s) on /icgnet/reconstruction_meshes.
[grasp_service_node-1] [INFO] [1781387575.913252649] [icgnet_grasp_node]: [INSTANCES] 4 instance(s): inst_0=ball(id=5, 870g), inst_1=cylindric(id=4, 1146g), inst_2=bottle(id=3, 1710g), inst_3=cylindric(id=4, 2958g) | total_grasps=6684
[grasp_service_node-1] [INFO] [1781387575.916121181] [icgnet_grasp_node]: [GRASP_POS] inst_0 ball (870g): mean=(0.693,0.168,0.209) x=[0.654,0.739] y=[0.118,0.193] z=[0.140,0.234]
[grasp_service_node-1] [INFO] [1781387575.918878535] [icgnet_grasp_node]: [GRASP_POS] inst_1 cylindric (1146g): mean=(0.691,0.167,0.199) x=[0.654,0.737] y=[0.100,0.194] z=[0.020,0.234]
[grasp_service_node-1] [INFO] [1781387575.923098320] [icgnet_grasp_node]: [GRASP_POS] inst_2 bottle (1710g): mean=(0.692,0.147,0.145) x=[0.653,0.745] y=[0.096,0.192] z=[0.033,0.234]
[grasp_service_node-1] [INFO] [1781387575.924956530] [icgnet_grasp_node]: [GRASP_POS] inst_3 cylindric (2958g): mean=(0.692,0.152,0.134) x=[0.654,0.742] y=[0.095,0.194] z=[0.016,0.234]
[grasp_service_node-1] [INFO] [1781387575.928381373] [icgnet_grasp_node]: [SCORES] top-10: [0.9607, 0.9564, 0.9564, 0.9559, 0.9552, 0.9535, 0.9534, 0.9532, 0.9523, 0.9521] | min=0.0000 max=0.9607 mean=0.5526 | >0.3: 4889 >0.5: 4467 >0.7: 2817
[grasp_service_node-1] [INFO] [1781387580.276564474] [icgnet_grasp_node]: [RECON_DIAG] 4 reconstruction(s) from ICGNet.
[grasp_service_node-1] [INFO] [1781387580.288856391] [icgnet_grasp_node]: [RECON_DIAG] inst=0: 3637v 7148f AABB=[0.677,0.103,0.087]->[0.733,0.169,0.144] centroid=(0.704,0.136,0.115)
[grasp_service_node-1] [INFO] [1781387580.319162348] [icgnet_grasp_node]: Published CollisionObject 'icgnet_inst_0' (628 triangles, hull=True).
[grasp_service_node-1] [INFO] [1781387580.326916322] [icgnet_grasp_node]: [RECON_DIAG] inst=1: 2272v 4342f AABB=[0.677,0.113,0.124]->[0.728,0.168,0.144] centroid=(0.706,0.138,0.133)
[grasp_service_node-1] [INFO] [1781387580.373205315] [icgnet_grasp_node]: Published CollisionObject 'icgnet_inst_1' (806 triangles, hull=True).
[grasp_service_node-1] [INFO] [1781387580.380957596] [icgnet_grasp_node]: [RECON_DIAG] inst=2: 3616v 7224f AABB=[0.666,0.099,0.031]->[0.734,0.170,0.143] centroid=(0.703,0.136,0.089)
[grasp_service_node-1] [INFO] [1781387580.418904969] [icgnet_grasp_node]: Published CollisionObject 'icgnet_inst_2' (950 triangles, hull=True).
[grasp_service_node-1] [INFO] [1781387580.427194013] [icgnet_grasp_node]: [RECON_DIAG] inst=3: 3200v 6356f AABB=[0.675,0.102,0.033]->[0.733,0.169,0.143] centroid=(0.707,0.136,0.091)
[grasp_service_node-1] [INFO] [1781387580.462558491] [icgnet_grasp_node]: Published CollisionObject 'icgnet_inst_3' (800 triangles, hull=True).
[grasp_service_node-1] [WARN] [1781387580.465735168] [icgnet_grasp_node]: [RECON_DIAG] AABB OVERLAP: inst 0 ↔ inst 1. Possible Mask3D under-segmentation or convex-hull inflation.
[grasp_service_node-1] [WARN] [1781387580.468205294] [icgnet_grasp_node]: [RECON_DIAG] AABB OVERLAP: inst 0 ↔ inst 2. Possible Mask3D under-segmentation or convex-hull inflation.
[grasp_service_node-1] [WARN] [1781387580.469741109] [icgnet_grasp_node]: [RECON_DIAG] AABB OVERLAP: inst 0 ↔ inst 3. Possible Mask3D under-segmentation or convex-hull inflation.
[grasp_service_node-1] [WARN] [1781387580.470942743] [icgnet_grasp_node]: [RECON_DIAG] AABB OVERLAP: inst 1 ↔ inst 2. Possible Mask3D under-segmentation or convex-hull inflation.
[grasp_service_node-1] [WARN] [1781387580.472019020] [icgnet_grasp_node]: [RECON_DIAG] AABB OVERLAP: inst 1 ↔ inst 3. Possible Mask3D under-segmentation or convex-hull inflation.
[grasp_service_node-1] [WARN] [1781387580.473019659] [icgnet_grasp_node]: [RECON_DIAG] AABB OVERLAP: inst 2 ↔ inst 3. Possible Mask3D under-segmentation or convex-hull inflation.
[grasp_service_node-1] [INFO] [1781387580.474662002] [icgnet_grasp_node]: Published 4 collision object(s) to MoveIt2.
[grasp_service_node-1] [INFO] [1781387580.492193653] [icgnet_grasp_node]: Published 6684 grasps
[grasp_service_node-1] [INFO] [1781387630.313063593] [icgnet_grasp_node]: Starting grasp computation...
[grasp_service_node-1] [INFO] [1781387631.944578126] [icgnet_grasp_node]: Bin exclusion: removed 21880 points.
[grasp_service_node-1] [INFO] [1781387632.082625552] [icgnet_grasp_node]: Preprocessing: 307200 → seg=7430 (encoder), grasp=1709 (sampling) points
[grasp_service_node-1] [INFO] [1781387632.112676551] [icgnet_predictor]: Running inference on 7430 points (return_meshes=True)...
[grasp_service_node-1] [INFO] [1781387634.033305400] [icgnet_predictor]: Raw grasp tensors: rot=torch.Size([3414, 3, 3]), centers=torch.Size([3414, 3]), scores=torch.Size([3414]), class_preds=torch.Size([2])
[grasp_service_node-1] [INFO] [1781387634.034276787] [icgnet_predictor]: Reconstructed 2 instance mesh(es).
[grasp_service_node-1] [INFO] [1781387634.035549789] [icgnet_predictor]: Inference complete.
[grasp_service_node-1] [INFO] [1781387634.042181607] [icgnet_grasp_node]: [RECON] inst_0 → class=cylindric (id=4)
[grasp_service_node-1] [INFO] [1781387634.043209910] [icgnet_grasp_node]: [RECON] inst_1 → class=cylindric (id=4)
[grasp_service_node-1] [INFO] [1781387634.393088953] [icgnet_grasp_node]: [RECON_VIZ] Published 2 reconstruction mesh marker(s) on /icgnet/reconstruction_meshes.
[grasp_service_node-1] [INFO] [1781387634.400125639] [icgnet_grasp_node]: [INSTANCES] 2 instance(s): inst_0=cylindric(id=4, 1326g), inst_1=cylindric(id=4, 2088g) | total_grasps=3414
[grasp_service_node-1] [INFO] [1781387634.401242385] [icgnet_grasp_node]: [GRASP_POS] inst_0 cylindric (1326g): mean=(0.800,0.231,0.190) x=[0.760,0.841] y=[0.183,0.256] z=[0.116,0.235]
[grasp_service_node-1] [INFO] [1781387634.402949560] [icgnet_grasp_node]: [GRASP_POS] inst_1 cylindric (2088g): mean=(0.801,0.229,0.178) x=[0.755,0.841] y=[0.182,0.256] z=[0.113,0.235]
[grasp_service_node-1] [INFO] [1781387634.404215791] [icgnet_grasp_node]: [SCORES] top-10: [0.9772, 0.977, 0.9767, 0.9766, 0.9764, 0.9763, 0.9762, 0.976, 0.9759, 0.9757] | min=0.5879 max=0.9772 mean=0.8448 | >0.3: 3414 >0.5: 3414 >0.7: 3268
[grasp_service_node-1] [INFO] [1781387636.405596809] [icgnet_grasp_node]: [RECON_DIAG] 2 reconstruction(s) from ICGNet.
[grasp_service_node-1] [INFO] [1781387636.415997910] [icgnet_grasp_node]: [RECON_DIAG] inst=0: 4214v 8416f AABB=[0.760,0.172,-0.012]->[0.822,0.229,0.131] centroid=(0.795,0.196,0.074)
[grasp_service_node-1] [INFO] [1781387636.451097446] [icgnet_grasp_node]: Published CollisionObject 'icgnet_inst_0' (928 triangles, hull=True).
[grasp_service_node-1] [INFO] [1781387636.458710279] [icgnet_grasp_node]: [RECON_DIAG] inst=1: 2822v 5628f AABB=[0.763,0.172,0.004]->[0.823,0.226,0.127] centroid=(0.800,0.193,0.068)
[grasp_service_node-1] [INFO] [1781387636.485689390] [icgnet_grasp_node]: Published CollisionObject 'icgnet_inst_1' (602 triangles, hull=True).
[grasp_service_node-1] [WARN] [1781387636.487158637] [icgnet_grasp_node]: [RECON_DIAG] AABB OVERLAP: inst 0 ↔ inst 1. Possible Mask3D under-segmentation or convex-hull inflation.
[grasp_service_node-1] [INFO] [1781387636.488592126] [icgnet_grasp_node]: Published 2 collision object(s) to MoveIt2.
[grasp_service_node-1] [INFO] [1781387636.498973444] [icgnet_grasp_node]: Published 3414 grasps
[grasp_service_node-1] [INFO] [1781387649.989143410] [icgnet_grasp_node]: Starting grasp computation...
[grasp_service_node-1] [INFO] [1781387651.514797360] [icgnet_grasp_node]: Bin exclusion: removed 18434 points.
[grasp_service_node-1] [INFO] [1781387651.641829399] [icgnet_grasp_node]: Preprocessing: 307200 → seg=7144 (encoder), grasp=1834 (sampling) points
[grasp_service_node-1] [INFO] [1781387651.670297298] [icgnet_predictor]: Running inference on 7144 points (return_meshes=True)...
[grasp_service_node-1] [INFO] [1781387654.416824659] [icgnet_predictor]: Raw grasp tensors: rot=torch.Size([5502, 3, 3]), centers=torch.Size([5502, 3]), scores=torch.Size([5502]), class_preds=torch.Size([3])
[grasp_service_node-1] [INFO] [1781387654.417952552] [icgnet_predictor]: Reconstructed 3 instance mesh(es).
[grasp_service_node-1] [INFO] [1781387654.418879712] [icgnet_predictor]: Inference complete.
[grasp_service_node-1] [INFO] [1781387654.427409437] [icgnet_grasp_node]: [RECON] inst_0 → class=can (id=2)
[grasp_service_node-1] [INFO] [1781387654.429476553] [icgnet_grasp_node]: [RECON] inst_1 → class=ball (id=5)
[grasp_service_node-1] [INFO] [1781387654.430857134] [icgnet_grasp_node]: [RECON] inst_2 → class=cylindric (id=4)
[grasp_service_node-1] [INFO] [1781387655.272103577] [icgnet_grasp_node]: [RECON_VIZ] Published 3 reconstruction mesh marker(s) on /icgnet/reconstruction_meshes.
[grasp_service_node-1] [INFO] [1781387655.285718502] [icgnet_grasp_node]: [INSTANCES] 3 instance(s): inst_0=can(id=2, 2322g), inst_1=ball(id=5, 3114g), inst_2=cylindric(id=4, 66g) | total_grasps=5502
[grasp_service_node-1] [INFO] [1781387655.288630982] [icgnet_grasp_node]: [GRASP_POS] inst_0 can (2322g): mean=(0.656,-0.261,0.165) x=[0.620,0.718] y=[-0.314,-0.220] z=[0.010,0.235]
[grasp_service_node-1] [INFO] [1781387655.290598737] [icgnet_grasp_node]: [GRASP_POS] inst_1 ball (3114g): mean=(0.660,-0.260,0.137) x=[0.620,0.718] y=[-0.314,-0.218] z=[0.010,0.235]
[grasp_service_node-1] [INFO] [1781387655.293135677] [icgnet_grasp_node]: [GRASP_POS] inst_2 cylindric (66g): mean=(0.664,-0.244,0.220) x=[0.647,0.679] y=[-0.288,-0.221] z=[0.191,0.235]
[grasp_service_node-1] [INFO] [1781387655.296809504] [icgnet_grasp_node]: [SCORES] top-10: [0.8531, 0.8506, 0.8418, 0.8416, 0.8384, 0.8381, 0.8378, 0.8378, 0.8374, 0.8352] | min=0.0000 max=0.8531 mean=0.4321 | >0.3: 4010 >0.5: 2502 >0.7: 1266
[grasp_service_node-1] [INFO] [1781387656.697013413] [icgnet_grasp_node]: [RECON_DIAG] 3 reconstruction(s) from ICGNet.
[grasp_service_node-1] [INFO] [1781387656.705360421] [icgnet_grasp_node]: [RECON_DIAG] inst=0: 2617v 5202f AABB=[0.652,-0.291,0.141]->[0.694,-0.248,0.197] centroid=(0.673,-0.268,0.181)
[grasp_service_node-1] [INFO] [1781387656.740564305] [icgnet_grasp_node]: Published CollisionObject 'icgnet_inst_0' (726 triangles, hull=True).
[grasp_service_node-1] [INFO] [1781387656.751338160] [icgnet_grasp_node]: [RECON_DIAG] inst=1: 4454v 8900f AABB=[0.647,-0.294,-0.003]->[0.703,-0.233,0.195] centroid=(0.677,-0.263,0.112)
[grasp_service_node-1] [INFO] [1781387656.786168301] [icgnet_grasp_node]: Published CollisionObject 'icgnet_inst_1' (788 triangles, hull=True).
[grasp_service_node-1] [INFO] [1781387656.795669948] [icgnet_grasp_node]: [RECON_DIAG] inst=2: 5308v 10576f AABB=[0.649,-0.293,-0.023]->[0.703,-0.235,0.196] centroid=(0.677,-0.264,0.091)
[grasp_service_node-1] [INFO] [1781387656.828430491] [icgnet_grasp_node]: Published CollisionObject 'icgnet_inst_2' (836 triangles, hull=True).
[grasp_service_node-1] [WARN] [1781387656.829675950] [icgnet_grasp_node]: [RECON_DIAG] AABB OVERLAP: inst 0 ↔ inst 1. Possible Mask3D under-segmentation or convex-hull inflation.
[grasp_service_node-1] [WARN] [1781387656.830878062] [icgnet_grasp_node]: [RECON_DIAG] AABB OVERLAP: inst 0 ↔ inst 2. Possible Mask3D under-segmentation or convex-hull inflation.
[grasp_service_node-1] [WARN] [1781387656.832011662] [icgnet_grasp_node]: [RECON_DIAG] AABB OVERLAP: inst 1 ↔ inst 2. Possible Mask3D under-segmentation or convex-hull inflation.
[grasp_service_node-1] [INFO] [1781387656.835332534] [icgnet_grasp_node]: Published 3 collision object(s) to MoveIt2.
[grasp_service_node-1] [INFO] [1781387656.844758904] [icgnet_grasp_node]: Published 5502 grasps
[grasp_service_node-1] [INFO] [1781387671.610457895] [icgnet_grasp_node]: Starting grasp computation...
[grasp_service_node-1] [INFO] [1781387673.164545498] [icgnet_grasp_node]: Bin exclusion: removed 21880 points.
[grasp_service_node-1] [INFO] [1781387673.266755402] [icgnet_grasp_node]: Preprocessing: 307200 → seg=8791 (encoder), grasp=2749 (sampling) points
[grasp_service_node-1] [INFO] [1781387673.295964953] [icgnet_predictor]: Running inference on 8791 points (return_meshes=True)...
[grasp_service_node-1] [INFO] [1781387674.597673082] [icgnet_predictor]: Raw grasp tensors: rot=torch.Size([2748, 3, 3]), centers=torch.Size([2748, 3]), scores=torch.Size([2748]), class_preds=torch.Size([1])
[grasp_service_node-1] [INFO] [1781387674.598817035] [icgnet_predictor]: Reconstructed 1 instance mesh(es).
[grasp_service_node-1] [INFO] [1781387674.599979041] [icgnet_predictor]: Inference complete.
[grasp_service_node-1] [INFO] [1781387674.606100758] [icgnet_grasp_node]: [RECON] inst_0 → class=box (id=1)
[grasp_service_node-1] [INFO] [1781387674.820235985] [icgnet_grasp_node]: [RECON_VIZ] Published 1 reconstruction mesh marker(s) on /icgnet/reconstruction_meshes.
[grasp_service_node-1] [INFO] [1781387674.826254585] [icgnet_grasp_node]: [INSTANCES] 1 instance(s): inst_0=box(id=1, 2748g) | total_grasps=2748
[grasp_service_node-1] [INFO] [1781387674.828378735] [icgnet_grasp_node]: [GRASP_POS] inst_0 box (2748g): mean=(0.405,0.222,0.173) x=[0.357,0.446] y=[0.171,0.264] z=[0.012,0.257]
[grasp_service_node-1] [INFO] [1781387674.829935262] [icgnet_grasp_node]: [SCORES] top-10: [0.8475, 0.8456, 0.8433, 0.8424, 0.839, 0.8355, 0.835, 0.8318, 0.8317, 0.831] | min=0.0000 max=0.8475 mean=0.4391 | >0.3: 1777 >0.5: 1679 >0.7: 877
[grasp_service_node-1] [INFO] [1781387676.367757650] [icgnet_grasp_node]: [RECON_DIAG] 1 reconstruction(s) from ICGNet.
[grasp_service_node-1] [INFO] [1781387676.376850649] [icgnet_grasp_node]: [RECON_DIAG] inst=0: 4046v 8060f AABB=[0.379,0.165,-0.042]->[0.440,0.244,0.130] centroid=(0.416,0.207,0.055)
[grasp_service_node-1] [INFO] [1781387676.405258838] [icgnet_grasp_node]: Published CollisionObject 'icgnet_inst_0' (574 triangles, hull=True).
[grasp_service_node-1] [INFO] [1781387676.406881420] [icgnet_grasp_node]: Published 1 collision object(s) to MoveIt2.
[grasp_service_node-1] [INFO] [1781387676.415584570] [icgnet_grasp_node]: Published 2748 grasps
[grasp_service_node-1] [INFO] [1781387691.124937896] [icgnet_grasp_node]: Starting grasp computation...
[grasp_service_node-1] [INFO] [1781387693.326595377] [icgnet_grasp_node]: Bin exclusion: removed 21880 points.
[grasp_service_node-1] [INFO] [1781387693.458300765] [icgnet_grasp_node]: Preprocessing: 307200 → seg=10690 (encoder), grasp=2677 (sampling) points
[grasp_service_node-1] [INFO] [1781387693.492778557] [icgnet_predictor]: Running inference on 10690 points (return_meshes=True)...
[grasp_service_node-1] [INFO] [1781387696.278258026] [icgnet_predictor]: Raw grasp tensors: rot=torch.Size([8028, 3, 3]), centers=torch.Size([8028, 3]), scores=torch.Size([8028]), class_preds=torch.Size([3])
[grasp_service_node-1] [INFO] [1781387696.279342861] [icgnet_predictor]: Reconstructed 3 instance mesh(es).
[grasp_service_node-1] [INFO] [1781387696.280375717] [icgnet_predictor]: Inference complete.
[grasp_service_node-1] [INFO] [1781387696.289123482] [icgnet_grasp_node]: [RECON] inst_0 → class=can (id=2)
[grasp_service_node-1] [INFO] [1781387696.290302174] [icgnet_grasp_node]: [RECON] inst_1 → class=can (id=2)
[grasp_service_node-1] [INFO] [1781387696.291439198] [icgnet_grasp_node]: [RECON] inst_2 → class=cylindric (id=4)
[grasp_service_node-1] [INFO] [1781387697.904016795] [icgnet_grasp_node]: [RECON_VIZ] Published 3 reconstruction mesh marker(s) on /icgnet/reconstruction_meshes.
[grasp_service_node-1] [INFO] [1781387697.924638318] [icgnet_grasp_node]: [INSTANCES] 3 instance(s): inst_0=can(id=2, 3876g), inst_1=can(id=2, 2424g), inst_2=cylindric(id=4, 1728g) | total_grasps=8028
[grasp_service_node-1] [INFO] [1781387697.928526863] [icgnet_grasp_node]: [GRASP_POS] inst_0 can (3876g): mean=(0.665,-0.047,0.083) x=[0.625,0.700] y=[-0.095,-0.002] z=[0.012,0.187]
[grasp_service_node-1] [INFO] [1781387697.931652750] [icgnet_grasp_node]: [GRASP_POS] inst_1 can (2424g): mean=(0.665,-0.047,0.077) x=[0.626,0.700] y=[-0.093,-0.002] z=[0.012,0.181]
[grasp_service_node-1] [INFO] [1781387697.933953053] [icgnet_grasp_node]: [GRASP_POS] inst_2 cylindric (1728g): mean=(0.668,-0.039,0.076) x=[0.625,0.698] y=[-0.094,-0.003] z=[0.012,0.257]
[grasp_service_node-1] [INFO] [1781387697.936216367] [icgnet_grasp_node]: [SCORES] top-10: [0.6264, 0.6197, 0.6151, 0.6045, 0.5943, 0.5908, 0.5905, 0.5872, 0.5855, 0.5832] | min=0.0000 max=0.6264 mean=0.0670 | >0.3: 856 >0.5: 240 >0.7: 0
[grasp_service_node-1] [INFO] [1781387702.649839186] [icgnet_grasp_node]: [RECON_DIAG] 3 reconstruction(s) from ICGNet.
[grasp_service_node-1] [INFO] [1781387702.664607682] [icgnet_grasp_node]: [RECON_DIAG] inst=0: 6776v 13544f AABB=[0.620,-0.094,-0.035]->[0.702,-0.007,0.218] centroid=(0.667,-0.050,0.127)
[grasp_service_node-1] [INFO] [1781387702.705177878] [icgnet_grasp_node]: Published CollisionObject 'icgnet_inst_0' (896 triangles, hull=True).
[grasp_service_node-1] [INFO] [1781387702.716379059] [icgnet_grasp_node]: [RECON_DIAG] inst=1: 6476v 12936f AABB=[0.620,-0.093,-0.034]->[0.701,-0.008,0.217] centroid=(0.668,-0.049,0.128)
[grasp_service_node-1] [INFO] [1781387702.761688992] [icgnet_grasp_node]: Published CollisionObject 'icgnet_inst_1' (1032 triangles, hull=True).
[grasp_service_node-1] [INFO] [1781387702.794644105] [icgnet_grasp_node]: [RECON_DIAG] inst=2: 12489v 24768f AABB=[0.633,-0.091,-0.028]->[0.701,-0.010,0.218] centroid=(0.673,-0.050,0.130)
[grasp_service_node-1] [INFO] [1781387702.831479863] [icgnet_grasp_node]: Published CollisionObject 'icgnet_inst_2' (676 triangles, hull=True).
[grasp_service_node-1] [WARN] [1781387702.833091863] [icgnet_grasp_node]: [RECON_DIAG] AABB OVERLAP: inst 0 ↔ inst 1. Possible Mask3D under-segmentation or convex-hull inflation.
[grasp_service_node-1] [WARN] [1781387702.834182537] [icgnet_grasp_node]: [RECON_DIAG] AABB OVERLAP: inst 0 ↔ inst 2. Possible Mask3D under-segmentation or convex-hull inflation.
[grasp_service_node-1] [WARN] [1781387702.834843985] [icgnet_grasp_node]: [RECON_DIAG] AABB OVERLAP: inst 1 ↔ inst 2. Possible Mask3D under-segmentation or convex-hull inflation.
[grasp_service_node-1] [INFO] [1781387702.836679135] [icgnet_grasp_node]: Published 3 collision object(s) to MoveIt2.
[grasp_service_node-1] [INFO] [1781387702.856156419] [icgnet_grasp_node]: Published 8028 grasps
[grasp_service_node-1] [INFO] [1781387748.423452828] [icgnet_grasp_node]: Starting grasp computation...
[grasp_service_node-1] [INFO] [1781387749.988500070] [icgnet_grasp_node]: Bin exclusion: removed 21880 points.
[grasp_service_node-1] [INFO] [1781387750.111511011] [icgnet_grasp_node]: Preprocessing: 307200 → seg=10719 (encoder), grasp=2581 (sampling) points
[grasp_service_node-1] [INFO] [1781387750.156574256] [icgnet_predictor]: Running inference on 10719 points (return_meshes=True)...
[grasp_service_node-1] [INFO] [1781387752.212486635] [icgnet_predictor]: Raw grasp tensors: rot=torch.Size([5160, 3, 3]), centers=torch.Size([5160, 3]), scores=torch.Size([5160]), class_preds=torch.Size([2])
[grasp_service_node-1] [INFO] [1781387752.213459843] [icgnet_predictor]: Reconstructed 2 instance mesh(es).
[grasp_service_node-1] [INFO] [1781387752.215116589] [icgnet_predictor]: Inference complete.
[grasp_service_node-1] [INFO] [1781387752.224519894] [icgnet_grasp_node]: [RECON] inst_0 → class=can (id=2)
[grasp_service_node-1] [INFO] [1781387752.225878094] [icgnet_grasp_node]: [RECON] inst_1 → class=can (id=2)
[grasp_service_node-1] [INFO] [1781387753.169633489] [icgnet_grasp_node]: [RECON_VIZ] Published 2 reconstruction mesh marker(s) on /icgnet/reconstruction_meshes.
[grasp_service_node-1] [INFO] [1781387753.181854752] [icgnet_grasp_node]: [INSTANCES] 2 instance(s): inst_0=can(id=2, 4338g), inst_1=can(id=2, 822g) | total_grasps=5160
[grasp_service_node-1] [INFO] [1781387753.184000692] [icgnet_grasp_node]: [GRASP_POS] inst_0 can (4338g): mean=(0.671,-0.045,0.112) x=[0.624,0.708] y=[-0.098,-0.005] z=[0.013,0.245]
[grasp_service_node-1] [INFO] [1781387753.185555034] [icgnet_grasp_node]: [GRASP_POS] inst_1 can (822g): mean=(0.669,-0.046,0.081) x=[0.631,0.705] y=[-0.091,-0.005] z=[0.014,0.210]
[grasp_service_node-1] [INFO] [1781387753.187441634] [icgnet_grasp_node]: [SCORES] top-10: [0.5944, 0.5823, 0.5635, 0.557, 0.555, 0.5543, 0.553, 0.5505, 0.5482, 0.5474] | min=0.0000 max=0.5944 mean=0.0923 | >0.3: 708 >0.5: 66 >0.7: 0
[grasp_service_node-1] [INFO] [1781387756.441054968] [icgnet_grasp_node]: [RECON_DIAG] 2 reconstruction(s) from ICGNet.
[grasp_service_node-1] [INFO] [1781387756.455573716] [icgnet_grasp_node]: [RECON_DIAG] inst=0: 7108v 14200f AABB=[0.624,-0.094,-0.037]->[0.706,-0.009,0.217] centroid=(0.672,-0.051,0.123)
[grasp_service_node-1] [INFO] [1781387756.515313697] [icgnet_grasp_node]: Published CollisionObject 'icgnet_inst_0' (996 triangles, hull=True).
[grasp_service_node-1] [INFO] [1781387756.528926767] [icgnet_grasp_node]: [RECON_DIAG] inst=1: 7008v 13980f AABB=[0.623,-0.094,-0.040]->[0.706,-0.009,0.217] centroid=(0.672,-0.051,0.122)
[grasp_service_node-1] [INFO] [1781387756.595102874] [icgnet_grasp_node]: Published CollisionObject 'icgnet_inst_1' (1076 triangles, hull=True).
[grasp_service_node-1] [WARN] [1781387756.597146832] [icgnet_grasp_node]: [RECON_DIAG] AABB OVERLAP: inst 0 ↔ inst 1. Possible Mask3D under-segmentation or convex-hull inflation.
[grasp_service_node-1] [INFO] [1781387756.603887106] [icgnet_grasp_node]: Published 2 collision object(s) to MoveIt2.
[grasp_service_node-1] [INFO] [1781387756.622674376] [icgnet_grasp_node]: Published 5160 grasps
[grasp_service_node-1] [INFO] [1781387772.502505831] [icgnet_grasp_node]: Starting grasp computation...
[grasp_service_node-1] [INFO] [1781387774.252701500] [icgnet_grasp_node]: Bin exclusion: removed 21880 points.
[grasp_service_node-1] [INFO] [1781387774.344694549] [icgnet_grasp_node]: Preprocessing: 307200 → seg=2254 (encoder), grasp=748 (sampling) points
[grasp_service_node-1] [INFO] [1781387774.357305127] [icgnet_predictor]: Running inference on 2254 points (return_meshes=True)...
[grasp_service_node-1] [INFO] [1781387775.345104401] [icgnet_predictor]: Raw grasp tensors: rot=torch.Size([744, 3, 3]), centers=torch.Size([744, 3]), scores=torch.Size([744]), class_preds=torch.Size([1])
[grasp_service_node-1] [INFO] [1781387775.346582787] [icgnet_predictor]: Reconstructed 1 instance mesh(es).
[grasp_service_node-1] [INFO] [1781387775.348748102] [icgnet_predictor]: Inference complete.
[grasp_service_node-1] [INFO] [1781387775.354606101] [icgnet_grasp_node]: [RECON] inst_0 → class=ball (id=5)
[grasp_service_node-1] [INFO] [1781387775.356858485] [icgnet_grasp_node]: [RECON_VIZ] Published 0 reconstruction mesh marker(s) on /icgnet/reconstruction_meshes.
[grasp_service_node-1] [INFO] [1781387775.358005519] [icgnet_grasp_node]: [INSTANCES] 1 instance(s): inst_0=ball(id=5, 744g) | total_grasps=744
[grasp_service_node-1] [INFO] [1781387775.359270092] [icgnet_grasp_node]: [GRASP_POS] inst_0 ball (744g): mean=(0.484,-0.138,0.069) x=[0.447,0.527] y=[-0.179,-0.094] z=[0.048,0.083]
[grasp_service_node-1] [INFO] [1781387775.360422522] [icgnet_grasp_node]: [SCORES] top-10: [0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001] | min=0.0000 max=0.0001 mean=0.0000 | >0.3: 0 >0.5: 0 >0.7: 0
[grasp_service_node-1] [INFO] [1781387775.773059938] [icgnet_grasp_node]: [RECON_DIAG] 1 reconstruction(s) from ICGNet.
[grasp_service_node-1] [WARN] [1781387775.774191103] [icgnet_grasp_node]: Instance 0: empty mesh, skipping collision object.
[grasp_service_node-1] [INFO] [1781387775.776144298] [icgnet_grasp_node]: Published 0 collision object(s) to MoveIt2.
[grasp_service_node-1] [INFO] [1781387775.782118778] [icgnet_grasp_node]: Published 744 grasps
[grasp_service_node-1] [INFO] [1781387898.668147442] [icgnet_grasp_node]: Starting grasp computation...
[grasp_service_node-1] [INFO] [1781387900.403252655] [icgnet_grasp_node]: Bin exclusion: removed 21880 points.
[grasp_service_node-1] [INFO] [1781387900.506545555] [icgnet_grasp_node]: Preprocessing: 307200 → seg=2091 (encoder), grasp=698 (sampling) points
[grasp_service_node-1] [INFO] [1781387900.521183013] [icgnet_predictor]: Running inference on 2091 points (return_meshes=True)...
[grasp_service_node-1] [INFO] [1781387899.846035837] [icgnet_predictor]: Raw grasp tensors: rot=torch.Size([696, 3, 3]), centers=torch.Size([696, 3]), scores=torch.Size([696]), class_preds=torch.Size([1])
[grasp_service_node-1] [INFO] [1781387899.846801547] [icgnet_predictor]: Reconstructed 1 instance mesh(es).
[grasp_service_node-1] [INFO] [1781387899.847560495] [icgnet_predictor]: Inference complete.
[grasp_service_node-1] [INFO] [1781387899.850901747] [icgnet_grasp_node]: [RECON] inst_0 → class=ball (id=5)
[grasp_service_node-1] [INFO] [1781387899.875316248] [icgnet_grasp_node]: [RECON_VIZ] Published 1 reconstruction mesh marker(s) on /icgnet/reconstruction_meshes.
[grasp_service_node-1] [INFO] [1781387899.878400009] [icgnet_grasp_node]: [INSTANCES] 1 instance(s): inst_0=ball(id=5, 696g) | total_grasps=696
[grasp_service_node-1] [INFO] [1781387899.881428617] [icgnet_grasp_node]: [GRASP_POS] inst_0 ball (696g): mean=(0.446,-0.051,0.070) x=[0.407,0.489] y=[-0.096,-0.012] z=[0.049,0.083]
[grasp_service_node-1] [INFO] [1781387899.883275429] [icgnet_grasp_node]: [SCORES] top-10: [0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001] | min=0.0000 max=0.0001 mean=0.0000 | >0.3: 0 >0.5: 0 >0.7: 0
[grasp_service_node-1] [INFO] [1781387900.313241091] [icgnet_grasp_node]: [RECON_DIAG] 1 reconstruction(s) from ICGNet.
[grasp_service_node-1] [INFO] [1781387900.315755456] [icgnet_grasp_node]: [RECON_DIAG] inst=0: 404v 760f AABB=[0.431,-0.088,0.041]->[0.482,-0.024,0.068] centroid=(0.446,-0.063,0.054)
[grasp_service_node-1] [INFO] [1781387900.333594591] [icgnet_grasp_node]: Published CollisionObject 'icgnet_inst_0' (244 triangles, hull=True).
[grasp_service_node-1] [INFO] [1781387900.335418264] [icgnet_grasp_node]: Published 1 collision object(s) to MoveIt2.
[grasp_service_node-1] [INFO] [1781387900.337719621] [icgnet_grasp_node]: Published 696 grasps
[grasp_service_node-1] [INFO] [1781388528.450035926] [icgnet_grasp_node]: Starting grasp computation...
[grasp_service_node-1] [INFO] [1781388533.608771491] [icgnet_grasp_node]: Bin exclusion: removed 21880 points.
[grasp_service_node-1] [INFO] [1781388533.716106648] [icgnet_grasp_node]: Preprocessing: 307200 → seg=3085 (encoder), grasp=827 (sampling) points
[grasp_service_node-1] [INFO] [1781388533.777224031] [icgnet_predictor]: Running inference on 3085 points (return_meshes=True)...
[grasp_service_node-1] [INFO] [1781388535.804099435] [icgnet_predictor]: Raw grasp tensors: rot=torch.Size([822, 3, 3]), centers=torch.Size([822, 3]), scores=torch.Size([822]), class_preds=torch.Size([1])
[grasp_service_node-1] [INFO] [1781388535.805017498] [icgnet_predictor]: Reconstructed 1 instance mesh(es).
[grasp_service_node-1] [INFO] [1781388535.805769555] [icgnet_predictor]: Inference complete.
[grasp_service_node-1] [INFO] [1781388535.812667526] [icgnet_grasp_node]: [RECON] inst_0 → class=other (id=6)
[grasp_service_node-1] [INFO] [1781388535.932375766] [icgnet_grasp_node]: [RECON_VIZ] Published 1 reconstruction mesh marker(s) on /icgnet/reconstruction_meshes.
[grasp_service_node-1] [INFO] [1781388535.935958605] [icgnet_grasp_node]: [INSTANCES] 1 instance(s): inst_0=other(id=6, 822g) | total_grasps=822
[grasp_service_node-1] [INFO] [1781388535.939649183] [icgnet_grasp_node]: [GRASP_POS] inst_0 other (822g): mean=(0.670,0.090,0.062) x=[0.619,0.706] y=[0.031,0.121] z=[0.033,0.083]
[grasp_service_node-1] [INFO] [1781388535.942001547] [icgnet_grasp_node]: [SCORES] top-10: [0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001] | min=0.0000 max=0.0001 mean=0.0000 | >0.3: 0 >0.5: 0 >0.7: 0
[grasp_service_node-1] [INFO] [1781388536.454877515] [icgnet_grasp_node]: [RECON_DIAG] 1 reconstruction(s) from ICGNet.
[grasp_service_node-1] [INFO] [1781388536.461334994] [icgnet_grasp_node]: [RECON_DIAG] inst=0: 1908v 3812f AABB=[0.633,0.041,0.048]->[0.695,0.107,0.077] centroid=(0.664,0.074,0.058)
[grasp_service_node-1] [INFO] [1781388536.515600807] [icgnet_grasp_node]: Published CollisionObject 'icgnet_inst_0' (1170 triangles, hull=True).
[grasp_service_node-1] [INFO] [1781388536.518244608] [icgnet_grasp_node]: Published 1 collision object(s) to MoveIt2.
[grasp_service_node-1] [INFO] [1781388536.524171653] [icgnet_grasp_node]: Published 822 grasps