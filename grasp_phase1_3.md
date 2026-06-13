alber@DESKTOP-L4POG0B:~/Robotics_Project/instance-centric-grasping$ ros2 launch icgnet_main grasp_execution.launch.py
[INFO] [launch]: All log files can be found below /home/alber/.ros/log/2026-06-13-23-41-17-880287-DESKTOP-L4POG0B-136755
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [grasp_executor-1]: process started with pid [136762]
[grasp_executor-1] [INFO] [1781386882.712429213] [grasp_executor_node]: GraspExecutorNode ready.
[grasp_executor-1] [INFO] [1781386903.137097040] [grasp_executor_node]: ExecuteGrasp: target='mug' max_attempts=5 skip_place=False
[grasp_executor-1] [INFO] [1781386903.170583227] [grasp_executor_node]: [BIN CO] Published drop_bin at (0.45, -0.50), footprint=0.30m, rim_h=0.10m
[grasp_executor-1] [INFO] [1781386903.192990140] [grasp_executor_node]: [RESET] Moving arm to home before inference (clean scene for ICGNet)...
[grasp_executor-1] [INFO] [1781386903.237227062] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781386903.244841521] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781386903.245749401] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781386910.332288383] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781386910.339598641] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781386910.340886606] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781386910.570329580] [grasp_executor_node]: [RESET] Arm at home.
[grasp_executor-1] [INFO] [1781386932.319304915] [grasp_executor_node]: [FILTER] total=7950 → kept=0 (scores=—) | rejected: width=0 workspace=0 target=7950 low_prepos=0
[grasp_executor-1] [INFO] [1781386945.262051646] [grasp_executor_node]: ExecuteGrasp: target='mug' max_attempts=5 skip_place=False
[grasp_executor-1] [INFO] [1781386945.266381353] [grasp_executor_node]: [RESET] Moving arm to home before inference (clean scene for ICGNet)...
[grasp_executor-1] [INFO] [1781386945.267156380] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781386945.283385469] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781386945.299934163] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781386945.509331959] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781386945.545431180] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781386945.546452956] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781386945.768360414] [grasp_executor_node]: [RESET] Arm at home.
[grasp_executor-1] [INFO] [1781386955.475457264] [grasp_executor_node]: [FILTER] total=8106 → kept=0 (scores=—) | rejected: width=0 workspace=4 target=8102 low_prepos=0
[grasp_executor-1] [INFO] [1781386970.282768915] [grasp_executor_node]: ExecuteGrasp: target='mug' max_attempts=5 skip_place=False
[grasp_executor-1] [INFO] [1781386970.294425957] [grasp_executor_node]: [RESET] Moving arm to home before inference (clean scene for ICGNet)...
[grasp_executor-1] [INFO] [1781386970.316613184] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781386970.341197954] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781386970.357848969] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781386970.592338910] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781386970.628047359] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781386970.629433975] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781386970.971827591] [grasp_executor_node]: [RESET] Arm at home.
[grasp_executor-1] [INFO] [1781386986.654295782] [grasp_executor_node]: [FILTER] total=9432 → kept=0 (scores=—) | rejected: width=0 workspace=4 target=9428 low_prepos=0
[grasp_executor-1] [INFO] [1781386997.875133042] [grasp_executor_node]: ExecuteGrasp: target='box' max_attempts=5 skip_place=False
[grasp_executor-1] [INFO] [1781386997.917990534] [grasp_executor_node]: [RESET] Moving arm to home before inference (clean scene for ICGNet)...
[grasp_executor-1] [INFO] [1781386997.945604701] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781386997.982838439] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781386998.017552546] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781386998.248946954] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781386998.250084748] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781386998.250879249] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781386998.473435766] [grasp_executor_node]: [RESET] Arm at home.
[grasp_executor-1] [INFO] [1781387002.599050707] [grasp_executor_node]: [FILTER] total=1380 → kept=685 (scores=[0.00–0.34]) | rejected: width=0 workspace=0 target=0 low_prepos=695
[grasp_executor-1] [INFO] [1781387002.612245101] [grasp_executor_node]: [SELECT] 5 candidates:
[grasp_executor-1] [INFO] [1781387002.639022525] [grasp_executor_node]:   [1] score=0.34 inst=0 cls=1(box) pos=[0.673,0.058,0.088] approach=[-0.478,-0.693,-0.541] angle_from_vertical=57.3° width=0.080
[grasp_executor-1] [INFO] [1781387002.651429711] [grasp_executor_node]:   [2] score=0.33 inst=0 cls=1(box) pos=[0.669,0.051,0.098] approach=[-0.372,-0.539,-0.756] angle_from_vertical=40.9° width=0.080
[grasp_executor-1] [INFO] [1781387002.652578180] [grasp_executor_node]:   [3] score=0.32 inst=0 cls=1(box) pos=[0.646,0.018,0.085] approach=[0.545,0.790,-0.282] angle_from_vertical=73.6° width=0.080
[grasp_executor-1] [INFO] [1781387002.653587133] [grasp_executor_node]:   [4] score=0.32 inst=0 cls=1(box) pos=[0.675,0.060,0.087] approach=[-0.478,-0.693,-0.541] angle_from_vertical=57.3° width=0.080
[grasp_executor-1] [INFO] [1781387002.654575159] [grasp_executor_node]:   [5] score=0.31 inst=0 cls=1(box) pos=[0.646,0.018,0.082] approach=[0.545,0.790,-0.282] angle_from_vertical=73.6° width=0.080
[grasp_executor-1] [INFO] [1781387002.655636113] [grasp_executor_node]: [RESET] Teleport-back target = [0.666, 0.047, 0.045] (from /model_poses)
[grasp_executor-1] [INFO] [1781387002.657465145] [grasp_executor_node]: ============================================================
[grasp_executor-1] [ATTEMPT 1/5] score=0.34 inst=0 cls=1(box) pos=[0.673,0.058,0.088]
[grasp_executor-1] [INFO] [1781387002.685280059] [grasp_executor_node]: [PLAN] score=0.342  inst=0  cls=1(box)  width=0.0800m
[grasp_executor-1]        icgnet_tcp  = [0.6734, 0.0578, 0.0879]
[grasp_executor-1]        contact_pos = [0.6519, 0.0266, 0.0635]  (forward_offset=4.5cm)
[grasp_executor-1]        pre_pos     = [0.7307, 0.1409, 0.1528]
[grasp_executor-1]        lift_pos    = [0.6519, 0.0266, 0.2435]
[grasp_executor-1]        approach    = [-0.4776, -0.6925, -0.5406]  (57.3° from vertical)
[grasp_executor-1] [INFO] [1781387002.699275083] [grasp_executor_node]: [GRIPPER] pre-grasp opening=80.0mm (ICGNet=80.0mm, per finger=40.0mm) [CAPPED at max_finger_pos=40mm/side]
[grasp_executor-1] [WARN] [1781387002.737644582] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387002.753557633] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387003.088101614] [grasp_executor_node]: [STEP 1/5] PRE-GRASP → [0.731, 0.141, 0.153]
[grasp_executor-1] [WARN] [1781387003.103540839] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387003.116913749] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [WARN] [1781387008.316245743] [grasp_executor_node]: Planning failed! Error code: FAILURE
[grasp_executor-1] [WARN] [1781387008.373275373] [grasp_executor_node]: Cannot execute motion because the provided/planned trajectory is invalid.
[grasp_executor-1] [WARN] [1781387008.381566433] [grasp_executor_node]: Cannot wait until motion is executed (no motion is in progress).
[grasp_executor-1] [WARN] [1781387008.382611114] [grasp_executor_node]: [STEP 1/5] PRE-GRASP FAILED in 5.29s — aborting this candidate
[grasp_executor-1] [WARN] [1781387008.383664356] [grasp_executor_node]: [ATTEMPT 1/5] FAILED (PREGRASP_PLAN_FAIL) — resetting scene and trying next candidate
[grasp_executor-1] [INFO] [1781387008.580877866] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781387008.607236937] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387008.631412677] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387008.882772191] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781387008.893877217] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387008.895003396] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387009.132301954] [grasp_executor_node]: [RESET] Arm at home.
[grasp_executor-1] [INFO] [1781387009.173199133] [grasp_executor_node]: [RESET] Object undisturbed (<3cm from target) — skipping teleport.
[grasp_executor-1] [INFO] [1781387009.343519656] [grasp_executor_node]: [RESET] Re-added CO 'icgnet_inst_0' to scene.
[grasp_executor-1] [INFO] [1781387009.344450069] [grasp_executor_node]: ============================================================
[grasp_executor-1] [ATTEMPT 2/5] score=0.33 inst=0 cls=1(box) pos=[0.669,0.051,0.098]
[grasp_executor-1] [INFO] [1781387009.356601151] [grasp_executor_node]: [PLAN] score=0.332  inst=0  cls=1(box)  width=0.0800m
[grasp_executor-1]        icgnet_tcp  = [0.6687, 0.0509, 0.0976]
[grasp_executor-1]        contact_pos = [0.6519, 0.0266, 0.0635]  (forward_offset=4.5cm)
[grasp_executor-1]        pre_pos     = [0.7133, 0.1155, 0.1882]
[grasp_executor-1]        lift_pos    = [0.6519, 0.0266, 0.2435]
[grasp_executor-1]        approach    = [-0.3718, -0.5391, -0.7557]  (40.9° from vertical)
[grasp_executor-1] [INFO] [1781387009.379233283] [grasp_executor_node]: [GRIPPER] pre-grasp opening=80.0mm (ICGNet=80.0mm, per finger=40.0mm) [CAPPED at max_finger_pos=40mm/side]
[grasp_executor-1] [WARN] [1781387009.386627227] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387009.418141204] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387009.757408722] [grasp_executor_node]: [STEP 1/5] PRE-GRASP → [0.713, 0.116, 0.188]
[grasp_executor-1] [WARN] [1781387009.784321581] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387009.785435498] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [WARN] [1781387015.037518556] [grasp_executor_node]: Planning failed! Error code: FAILURE
[grasp_executor-1] [WARN] [1781387015.038382567] [grasp_executor_node]: Cannot execute motion because the provided/planned trajectory is invalid.
[grasp_executor-1] [WARN] [1781387015.068840962] [grasp_executor_node]: Cannot wait until motion is executed (no motion is in progress).
[grasp_executor-1] [WARN] [1781387015.089915601] [grasp_executor_node]: [STEP 1/5] PRE-GRASP FAILED in 5.31s — aborting this candidate
[grasp_executor-1] [WARN] [1781387015.095954167] [grasp_executor_node]: [ATTEMPT 2/5] FAILED (PREGRASP_PLAN_FAIL) — resetting scene and trying next candidate
[grasp_executor-1] [INFO] [1781387015.274599636] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781387015.275558439] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387015.293208526] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387015.501001579] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781387015.512716135] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387015.513603394] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387015.725508214] [grasp_executor_node]: [RESET] Arm at home.
[grasp_executor-1] [INFO] [1781387015.726764813] [grasp_executor_node]: [RESET] Object undisturbed (<3cm from target) — skipping teleport.
[grasp_executor-1] [INFO] [1781387015.910446507] [grasp_executor_node]: [RESET] Re-added CO 'icgnet_inst_0' to scene.
[grasp_executor-1] [INFO] [1781387015.924394746] [grasp_executor_node]: ============================================================
[grasp_executor-1] [ATTEMPT 3/5] score=0.32 inst=0 cls=1(box) pos=[0.646,0.018,0.085]
[grasp_executor-1] [INFO] [1781387015.934389206] [grasp_executor_node]: [PLAN] score=0.325  inst=0  cls=1(box)  width=0.0800m
[grasp_executor-1]        icgnet_tcp  = [0.6458, 0.0178, 0.0848]
[grasp_executor-1]        contact_pos = [0.6703, 0.0533, 0.0721]  (forward_offset=4.5cm)
[grasp_executor-1]        pre_pos     = [0.5805, -0.0770, 0.1186]
[grasp_executor-1]        lift_pos    = [0.6703, 0.0533, 0.2521]
[grasp_executor-1]        approach    = [0.5448, 0.7898, -0.2817]  (73.6° from vertical)
[grasp_executor-1] [INFO] [1781387015.935683849] [grasp_executor_node]: [GRIPPER] pre-grasp opening=80.0mm (ICGNet=80.0mm, per finger=40.0mm) [CAPPED at max_finger_pos=40mm/side]
[grasp_executor-1] [WARN] [1781387015.936786371] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387015.957224099] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387016.266603960] [grasp_executor_node]: [STEP 1/5] PRE-GRASP → [0.580, -0.077, 0.119]
[grasp_executor-1] [WARN] [1781387016.268084804] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387016.286573540] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387022.055574153] [grasp_executor_node]: [STEP 1/5] Pre-grasp reached in 5.72s
[grasp_executor-1] [INFO] [1781387022.948925919] [grasp_executor_node]: [CO] Removed 'icgnet_inst_0' from scene for approach
[grasp_executor-1] [INFO] [1781387022.950120738] [grasp_executor_node]: [STEP 2/5] CARTESIAN APPROACH → [0.670, 0.053, 0.072]  (vel=0.08)
[grasp_executor-1] [WARN] [1781387022.951464368] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387022.953367729] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387025.349236121] [grasp_executor_node]: [STEP 2/5] Contact surface reached in 2.39s
[grasp_executor-1] [INFO] [1781387025.382600590] [grasp_executor_node]: [STEP 3/5] CLOSING GRIPPER
[grasp_executor-1] [WARN] [1781387025.390086257] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387025.391037674] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387026.762379778] [grasp_executor_node]: [STEP 3/5] Object confirmed between fingers (gap=34.5mm, range [5–40mm])
[grasp_executor-1] [INFO] [1781387026.919789527] [grasp_executor_node]: [STEP 3b] Attached 'icgnet_inst_0' to panda_hand_tcp
[grasp_executor-1] [INFO] [1781387026.959593126] [grasp_executor_node]: [STEP 4/5] CARTESIAN LIFT → [0.670, 0.053, 0.252]
[grasp_executor-1] [WARN] [1781387026.961141054] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387026.962121218] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387030.559562453] [grasp_executor_node]: [STEP 4/5] Object lifted in 3.58s
[grasp_executor-1] [INFO] [1781387030.598667047] [grasp_executor_node]: [STEP 5/7] TRANSFER → ['0.450', '-0.500', '0.350']  (joint-space, orient=target, vel=0.25)
[grasp_executor-1] [WARN] [1781387030.606559274] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387030.632722959] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [WARN] [1781387030.976406331] [grasp_executor_node]: Planning failed! Error code: INVALID_MOTION_PLAN
[grasp_executor-1] [WARN] [1781387030.977138662] [grasp_executor_node]: Cannot execute motion because the provided/planned trajectory is invalid.
[grasp_executor-1] [WARN] [1781387030.978041595] [grasp_executor_node]: Cannot wait until motion is executed (no motion is in progress).
[grasp_executor-1] [WARN] [1781387030.979132475] [grasp_executor_node]: [STEP 5/7] TRANSFER FAILED in 0.38s — aborting
[grasp_executor-1] [WARN] [1781387031.006933116] [grasp_executor_node]: [ATTEMPT 3/5] FAILED (TRANSFER_PLAN_FAIL) — resetting scene and trying next candidate
[grasp_executor-1] [INFO] [1781387031.187115746] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781387031.223755017] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387031.225129776] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387031.649890948] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781387031.676291868] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387031.677966371] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387038.835953676] [grasp_executor_node]: [RESET] Arm at home.
[grasp_executor-1] [INFO] [1781387038.837076973] [grasp_executor_node]: [RESET] Object undisturbed (<3cm from target) — skipping teleport.
[grasp_executor-1] [INFO] [1781387039.021150383] [grasp_executor_node]: [RESET] Re-added CO 'icgnet_inst_0' to scene.
[grasp_executor-1] [INFO] [1781387039.046998263] [grasp_executor_node]: ============================================================
[grasp_executor-1] [ATTEMPT 4/5] score=0.32 inst=0 cls=1(box) pos=[0.675,0.060,0.087]
[grasp_executor-1] [INFO] [1781387039.049868934] [grasp_executor_node]: [PLAN] score=0.317  inst=0  cls=1(box)  width=0.0800m
[grasp_executor-1]        icgnet_tcp  = [0.6750, 0.0600, 0.0872]
[grasp_executor-1]        contact_pos = [0.6535, 0.0288, 0.0629]  (forward_offset=4.5cm)
[grasp_executor-1]        pre_pos     = [0.7323, 0.1431, 0.1521]
[grasp_executor-1]        lift_pos    = [0.6535, 0.0288, 0.2429]
[grasp_executor-1]        approach    = [-0.4776, -0.6925, -0.5406]  (57.3° from vertical)
[grasp_executor-1] [INFO] [1781387039.051204837] [grasp_executor_node]: [GRIPPER] pre-grasp opening=80.0mm (ICGNet=80.0mm, per finger=40.0mm) [CAPPED at max_finger_pos=40mm/side]
[grasp_executor-1] [WARN] [1781387039.061066591] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387039.067095285] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387039.394645259] [grasp_executor_node]: [STEP 1/5] PRE-GRASP → [0.732, 0.143, 0.152]
[grasp_executor-1] [WARN] [1781387039.440921495] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387039.477351095] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [WARN] [1781387044.742671432] [grasp_executor_node]: Planning failed! Error code: FAILURE
[grasp_executor-1] [WARN] [1781387044.750974525] [grasp_executor_node]: Cannot execute motion because the provided/planned trajectory is invalid.
[grasp_executor-1] [WARN] [1781387044.787130516] [grasp_executor_node]: Cannot wait until motion is executed (no motion is in progress).
[grasp_executor-1] [WARN] [1781387044.819670390] [grasp_executor_node]: [STEP 1/5] PRE-GRASP FAILED in 5.39s — aborting this candidate
[grasp_executor-1] [WARN] [1781387044.869478376] [grasp_executor_node]: [ATTEMPT 4/5] FAILED (PREGRASP_PLAN_FAIL) — resetting scene and trying next candidate
[grasp_executor-1] [INFO] [1781387045.100275966] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781387045.101666634] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387045.102857170] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387045.426699049] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781387045.427851631] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387045.428976408] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387045.762077953] [grasp_executor_node]: [RESET] Arm at home.
[grasp_executor-1] [INFO] [1781387045.763472532] [grasp_executor_node]: [RESET] Object undisturbed (<3cm from target) — skipping teleport.
[grasp_executor-1] [INFO] [1781387045.940370692] [grasp_executor_node]: [RESET] Re-added CO 'icgnet_inst_0' to scene.
[grasp_executor-1] [INFO] [1781387045.948030044] [grasp_executor_node]: ============================================================
[grasp_executor-1] [ATTEMPT 5/5] score=0.31 inst=0 cls=1(box) pos=[0.646,0.018,0.082]
[grasp_executor-1] [INFO] [1781387045.996168890] [grasp_executor_node]: [PLAN] score=0.312  inst=0  cls=1(box)  width=0.0800m
[grasp_executor-1]        icgnet_tcp  = [0.6458, 0.0177, 0.0819]
[grasp_executor-1]        contact_pos = [0.6703, 0.0532, 0.0692]  (forward_offset=4.5cm)
[grasp_executor-1]        pre_pos     = [0.5804, -0.0771, 0.1157]
[grasp_executor-1]        lift_pos    = [0.6703, 0.0532, 0.2492]
[grasp_executor-1]        approach    = [0.5448, 0.7898, -0.2817]  (73.6° from vertical)
[grasp_executor-1] [INFO] [1781387046.041216212] [grasp_executor_node]: [GRIPPER] pre-grasp opening=80.0mm (ICGNet=80.0mm, per finger=40.0mm) [CAPPED at max_finger_pos=40mm/side]
[grasp_executor-1] [WARN] [1781387046.064261400] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387046.090702591] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387046.426861955] [grasp_executor_node]: [STEP 1/5] PRE-GRASP → [0.580, -0.077, 0.116]
[grasp_executor-1] [WARN] [1781387046.449531659] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387046.450551878] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387053.282797263] [grasp_executor_node]: [STEP 1/5] Pre-grasp reached in 6.84s
[grasp_executor-1] [INFO] [1781387054.130106928] [grasp_executor_node]: [CO] Removed 'icgnet_inst_0' from scene for approach
[grasp_executor-1] [INFO] [1781387054.135486579] [grasp_executor_node]: [STEP 2/5] CARTESIAN APPROACH → [0.670, 0.053, 0.069]  (vel=0.08)
[grasp_executor-1] [WARN] [1781387054.159017099] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387054.160053585] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387055.794776772] [grasp_executor_node]: [STEP 2/5] Contact surface reached in 1.66s
[grasp_executor-1] [INFO] [1781387055.795446789] [grasp_executor_node]: [STEP 3/5] CLOSING GRIPPER
[grasp_executor-1] [WARN] [1781387055.796631731] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387055.797633044] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [WARN] [1781387056.634395002] [grasp_executor_node]: [STEP 3/5] Gripper fully closed (gap=0.2mm < 5mm) — missed object
[grasp_executor-1] [WARN] [1781387056.676334307] [grasp_executor_node]: [ATTEMPT 5/5] FAILED (GRASP_MISS) — no more candidates
[grasp_executor-1] [INFO] [1781387069.705254753] [grasp_executor_node]: ExecuteGrasp: target='box' max_attempts=5 skip_place=False
[grasp_executor-1] [INFO] [1781387069.707822782] [grasp_executor_node]: [RESET] Moving arm to home before inference (clean scene for ICGNet)...
[grasp_executor-1] [INFO] [1781387069.898416963] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781387069.936493539] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387069.940401969] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387070.868774903] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781387070.869753394] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387070.870582617] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387077.334376784] [grasp_executor_node]: [RESET] Arm at home.
[grasp_executor-1] [INFO] [1781387077.528961878] [grasp_executor_node]: [RESET] Re-added CO 'icgnet_inst_0' to scene.
[grasp_executor-1] [INFO] [1781387082.056317829] [grasp_executor_node]: [FILTER] total=1368 → kept=516 (scores=[0.00–0.12]) | rejected: width=0 workspace=0 target=0 low_prepos=852
[grasp_executor-1] [INFO] [1781387082.060753709] [grasp_executor_node]: [SELECT] 5 candidates:
[grasp_executor-1] [INFO] [1781387082.062839850] [grasp_executor_node]:   [1] score=0.12 inst=0 cls=1(box) pos=[0.548,0.167,0.076] approach=[0.488,0.685,-0.541] angle_from_vertical=57.3° width=0.080
[grasp_executor-1] [INFO] [1781387082.076125074] [grasp_executor_node]:   [2] score=0.00 inst=0 cls=1(box) pos=[0.580,0.211,0.073] approach=[-0.083,-0.116,-0.990] angle_from_vertical=8.2° width=0.080
[grasp_executor-1] [INFO] [1781387082.080932303] [grasp_executor_node]:   [3] score=0.00 inst=0 cls=1(box) pos=[0.573,0.201,0.073] approach=[0.083,0.116,-0.990] angle_from_vertical=8.2° width=0.080
[grasp_executor-1] [INFO] [1781387082.082131689] [grasp_executor_node]:   [4] score=0.00 inst=0 cls=1(box) pos=[0.598,0.237,0.052] approach=[-0.488,-0.685,-0.541] angle_from_vertical=57.3° width=0.080
[grasp_executor-1] [INFO] [1781387082.083236476] [grasp_executor_node]:   [5] score=0.00 inst=0 cls=1(box) pos=[0.599,0.237,0.050] approach=[-0.488,-0.685,-0.541] angle_from_vertical=57.3° width=0.080
[grasp_executor-1] [INFO] [1781387082.084122284] [grasp_executor_node]: [RESET] Teleport-back target = [0.587, 0.226, 0.045] (from /model_poses)
[grasp_executor-1] [INFO] [1781387082.085304624] [grasp_executor_node]: ============================================================
[grasp_executor-1] [ATTEMPT 1/5] score=0.12 inst=0 cls=1(box) pos=[0.548,0.167,0.076]
[grasp_executor-1] [INFO] [1781387082.089969269] [grasp_executor_node]: [PLAN] score=0.119  inst=0  cls=1(box)  width=0.0800m
[grasp_executor-1]        icgnet_tcp  = [0.5484, 0.1666, 0.0765]
[grasp_executor-1]        contact_pos = [0.5704, 0.1974, 0.0521]  (forward_offset=4.5cm)
[grasp_executor-1]        pre_pos     = [0.4899, 0.0844, 0.1413]
[grasp_executor-1]        lift_pos    = [0.5704, 0.1974, 0.2321]
[grasp_executor-1]        approach    = [0.4878, 0.6854, -0.5406]  (57.3° from vertical)
[grasp_executor-1] [INFO] [1781387082.116817623] [grasp_executor_node]: [GRIPPER] pre-grasp opening=80.0mm (ICGNet=80.0mm, per finger=40.0mm) [CAPPED at max_finger_pos=40mm/side]
[grasp_executor-1] [WARN] [1781387082.132916640] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387082.134132442] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387082.451201678] [grasp_executor_node]: [STEP 1/5] PRE-GRASP → [0.490, 0.084, 0.141]
[grasp_executor-1] [WARN] [1781387082.452670554] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387082.453838898] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387088.201449814] [grasp_executor_node]: [STEP 1/5] Pre-grasp reached in 5.74s
[grasp_executor-1] [INFO] [1781387089.045474018] [grasp_executor_node]: [CO] Removed 'icgnet_inst_0' from scene for approach
[grasp_executor-1] [INFO] [1781387089.095574010] [grasp_executor_node]: [STEP 2/5] CARTESIAN APPROACH → [0.570, 0.197, 0.052]  (vel=0.08)
[grasp_executor-1] [WARN] [1781387089.097101530] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387089.098337515] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387091.451578526] [grasp_executor_node]: [STEP 2/5] Contact surface reached in 2.36s
[grasp_executor-1] [INFO] [1781387091.474360062] [grasp_executor_node]: [STEP 3/5] CLOSING GRIPPER
[grasp_executor-1] [WARN] [1781387091.514157691] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387091.515176891] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [WARN] [1781387092.467350156] [grasp_executor_node]: [STEP 3/5] Gripper fully closed (gap=0.4mm < 5mm) — missed object
[grasp_executor-1] [WARN] [1781387092.507665382] [grasp_executor_node]: [ATTEMPT 1/5] FAILED (GRASP_MISS) — resetting scene and trying next candidate
[grasp_executor-1] [INFO] [1781387092.665330832] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781387092.666540799] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387092.676918018] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387093.624181999] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781387093.646839627] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387093.648212422] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387100.515162010] [grasp_executor_node]: [RESET] Arm at home.
[grasp_executor-1] [INFO] [1781387100.524168609] [grasp_executor_node]: [RESET] Object undisturbed (<3cm from target) — skipping teleport.
[grasp_executor-1] [INFO] [1781387100.695879558] [grasp_executor_node]: [RESET] Re-added CO 'icgnet_inst_0' to scene.
[grasp_executor-1] [INFO] [1781387100.722704915] [grasp_executor_node]: ============================================================
[grasp_executor-1] [ATTEMPT 2/5] score=0.00 inst=0 cls=1(box) pos=[0.580,0.211,0.073]
[grasp_executor-1] [INFO] [1781387100.736919795] [grasp_executor_node]: [PLAN] score=0.000  inst=0  cls=1(box)  width=0.0800m
[grasp_executor-1]        icgnet_tcp  = [0.5802, 0.2113, 0.0727]
[grasp_executor-1]        contact_pos = [0.5765, 0.2060, 0.0281]  (forward_offset=4.5cm)
[grasp_executor-1]        pre_pos     = [0.5901, 0.2252, 0.1915]
[grasp_executor-1]        lift_pos    = [0.5765, 0.2060, 0.2081]
[grasp_executor-1]        approach    = [-0.0825, -0.1159, -0.9898]  (8.2° from vertical)
[grasp_executor-1] [INFO] [1781387100.738513527] [grasp_executor_node]: [GRIPPER] pre-grasp opening=80.0mm (ICGNet=80.0mm, per finger=40.0mm) [CAPPED at max_finger_pos=40mm/side]
[grasp_executor-1] [WARN] [1781387100.739702689] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387100.750820817] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387101.100883796] [grasp_executor_node]: [STEP 1/5] PRE-GRASP → [0.590, 0.225, 0.191]
[grasp_executor-1] [WARN] [1781387101.116208974] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387101.116921532] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387107.074271824] [grasp_executor_node]: [STEP 1/5] Pre-grasp reached in 5.95s
[grasp_executor-1] [INFO] [1781387107.901542391] [grasp_executor_node]: [CO] Removed 'icgnet_inst_0' from scene for approach
[grasp_executor-1] [INFO] [1781387107.902380069] [grasp_executor_node]: [STEP 2/5] CARTESIAN APPROACH → [0.576, 0.206, 0.028]  (vel=0.08)
[grasp_executor-1] [WARN] [1781387107.903874083] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387107.905994877] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [ERROR] [1781387168.142300629] [grasp_executor_node]: wait_until_executed timed out after 60.0s — cancelling motion and resetting execution state.
[grasp_executor-1] [WARN] [1781387168.158854125] [grasp_executor_node]: [STEP 2/5] APPROACH FAILED in 60.24s — aborting this candidate
[grasp_executor-1] [WARN] [1781387168.159995778] [grasp_executor_node]: [ATTEMPT 2/5] FAILED (APPROACH_FAIL) — resetting scene and trying next candidate
[grasp_executor-1] [WARN] [1781387168.200703256] [grasp_executor_node]: Action 'execute_trajectory' was unsuccessful: STATUS_ABORTED.
[grasp_executor-1] [INFO] [1781387168.329124180] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781387168.352825877] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387168.368558305] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387168.678992002] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781387168.684033414] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387168.697475770] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387173.177296446] [grasp_executor_node]: [RESET] Arm at home.
[grasp_executor-1] [INFO] [1781387173.178236426] [grasp_executor_node]: [RESET] Object undisturbed (<3cm from target) — skipping teleport.
[grasp_executor-1] [INFO] [1781387173.359786239] [grasp_executor_node]: [RESET] Re-added CO 'icgnet_inst_0' to scene.
[grasp_executor-1] [INFO] [1781387173.360884556] [grasp_executor_node]: ============================================================
[grasp_executor-1] [ATTEMPT 3/5] score=0.00 inst=0 cls=1(box) pos=[0.573,0.201,0.073]
[grasp_executor-1] [INFO] [1781387173.370013342] [grasp_executor_node]: [PLAN] score=0.000  inst=0  cls=1(box)  width=0.0800m
[grasp_executor-1]        icgnet_tcp  = [0.5728, 0.2008, 0.0727]
[grasp_executor-1]        contact_pos = [0.5765, 0.2060, 0.0281]  (forward_offset=4.5cm)
[grasp_executor-1]        pre_pos     = [0.5629, 0.1869, 0.1915]
[grasp_executor-1]        lift_pos    = [0.5765, 0.2060, 0.2081]
[grasp_executor-1]        approach    = [0.0825, 0.1159, -0.9898]  (8.2° from vertical)
[grasp_executor-1] [INFO] [1781387173.414990526] [grasp_executor_node]: [GRIPPER] pre-grasp opening=80.0mm (ICGNet=80.0mm, per finger=40.0mm) [CAPPED at max_finger_pos=40mm/side]
[grasp_executor-1] [WARN] [1781387173.438390558] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387173.495836083] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387173.834740609] [grasp_executor_node]: [STEP 1/5] PRE-GRASP → [0.563, 0.187, 0.191]
[grasp_executor-1] [WARN] [1781387173.835722446] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387173.836382747] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387180.524820070] [grasp_executor_node]: [STEP 1/5] Pre-grasp reached in 6.67s
[grasp_executor-1] [INFO] [1781387181.361200853] [grasp_executor_node]: [CO] Removed 'icgnet_inst_0' from scene for approach
[grasp_executor-1] [INFO] [1781387181.373457255] [grasp_executor_node]: [STEP 2/5] CARTESIAN APPROACH → [0.576, 0.206, 0.028]  (vel=0.08)
[grasp_executor-1] [WARN] [1781387181.375544455] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387181.377982660] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387184.553707790] [grasp_executor_node]: [STEP 2/5] Contact surface reached in 3.16s
[grasp_executor-1] [INFO] [1781387184.554528867] [grasp_executor_node]: [STEP 3/5] CLOSING GRIPPER
[grasp_executor-1] [WARN] [1781387184.555739862] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387184.556830002] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [WARN] [1781387185.289560472] [grasp_executor_node]: [STEP 3/5] Gripper fully closed (gap=0.2mm < 5mm) — missed object
[grasp_executor-1] [WARN] [1781387185.300891117] [grasp_executor_node]: [ATTEMPT 3/5] FAILED (GRASP_MISS) — resetting scene and trying next candidate
[grasp_executor-1] [INFO] [1781387185.481092892] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781387185.508881857] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387185.510505882] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387186.352764262] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781387186.373807252] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387186.374856470] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387193.648841155] [grasp_executor_node]: [RESET] Arm at home.
[grasp_executor-1] [INFO] [1781387193.697052963] [grasp_executor_node]: [RESET] Object "target_obj" reset to [0.587, 0.226, 0.045]
[grasp_executor-1] [INFO] [1781387193.856435451] [grasp_executor_node]: [RESET] Re-added CO 'icgnet_inst_0' to scene.
[grasp_executor-1] [INFO] [1781387193.892524951] [grasp_executor_node]: ============================================================
[grasp_executor-1] [ATTEMPT 4/5] score=0.00 inst=0 cls=1(box) pos=[0.598,0.237,0.052]
[grasp_executor-1] [INFO] [1781387193.926653317] [grasp_executor_node]: [PLAN] score=0.000  inst=0  cls=1(box)  width=0.0800m
[grasp_executor-1]        icgnet_tcp  = [0.5984, 0.2369, 0.0525]
[grasp_executor-1]        contact_pos = [0.5765, 0.2060, 0.0281]  (forward_offset=4.5cm)
[grasp_executor-1]        pre_pos     = [0.6570, 0.3191, 0.1174]
[grasp_executor-1]        lift_pos    = [0.5765, 0.2060, 0.2081]
[grasp_executor-1]        approach    = [-0.4878, -0.6854, -0.5406]  (57.3° from vertical)
[grasp_executor-1] [INFO] [1781387193.938908096] [grasp_executor_node]: [GRIPPER] pre-grasp opening=80.0mm (ICGNet=80.0mm, per finger=40.0mm) [CAPPED at max_finger_pos=40mm/side]
[grasp_executor-1] [WARN] [1781387193.949544980] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387193.950519674] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387194.265504800] [grasp_executor_node]: [STEP 1/5] PRE-GRASP → [0.657, 0.319, 0.117]
[grasp_executor-1] [WARN] [1781387194.274751063] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387194.302510800] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [WARN] [1781387199.434500261] [grasp_executor_node]: Planning failed! Error code: FAILURE
[grasp_executor-1] [WARN] [1781387199.436286681] [grasp_executor_node]: Cannot execute motion because the provided/planned trajectory is invalid.
[grasp_executor-1] [WARN] [1781387199.437448140] [grasp_executor_node]: Cannot wait until motion is executed (no motion is in progress).
[grasp_executor-1] [WARN] [1781387199.451215055] [grasp_executor_node]: [STEP 1/5] PRE-GRASP FAILED in 5.17s — aborting this candidate
[grasp_executor-1] [WARN] [1781387199.453237382] [grasp_executor_node]: [ATTEMPT 4/5] FAILED (PREGRASP_PLAN_FAIL) — resetting scene and trying next candidate
[grasp_executor-1] [INFO] [1781387199.681813685] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781387199.706253991] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387199.738628765] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387200.074916852] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781387200.079030629] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387200.080722877] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387198.523807160] [grasp_executor_node]: [RESET] Arm at home.
[grasp_executor-1] [INFO] [1781387198.524534539] [grasp_executor_node]: [RESET] Object undisturbed (<3cm from target) — skipping teleport.
[grasp_executor-1] [INFO] [1781387198.691945866] [grasp_executor_node]: [RESET] Re-added CO 'icgnet_inst_0' to scene.
[grasp_executor-1] [INFO] [1781387198.693062617] [grasp_executor_node]: ============================================================
[grasp_executor-1] [ATTEMPT 5/5] score=0.00 inst=0 cls=1(box) pos=[0.599,0.237,0.050]
[grasp_executor-1] [INFO] [1781387198.696014090] [grasp_executor_node]: [PLAN] score=0.000  inst=0  cls=1(box)  width=0.0800m
[grasp_executor-1]        icgnet_tcp  = [0.5986, 0.2371, 0.0496]
[grasp_executor-1]        contact_pos = [0.5766, 0.2063, 0.0252]  (forward_offset=4.5cm)
[grasp_executor-1]        pre_pos     = [0.6571, 0.3193, 0.1144]
[grasp_executor-1]        lift_pos    = [0.5766, 0.2063, 0.2052]
[grasp_executor-1]        approach    = [-0.4878, -0.6854, -0.5406]  (57.3° from vertical)
[grasp_executor-1] [INFO] [1781387198.697429951] [grasp_executor_node]: [GRIPPER] pre-grasp opening=80.0mm (ICGNet=80.0mm, per finger=40.0mm) [CAPPED at max_finger_pos=40mm/side]
[grasp_executor-1] [WARN] [1781387198.715505012] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387198.716352774] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387199.028898550] [grasp_executor_node]: [STEP 1/5] PRE-GRASP → [0.657, 0.319, 0.114]
[grasp_executor-1] [WARN] [1781387199.037767130] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387199.039570779] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [WARN] [1781387204.192186974] [grasp_executor_node]: Planning failed! Error code: FAILURE
[grasp_executor-1] [WARN] [1781387204.207066518] [grasp_executor_node]: Cannot execute motion because the provided/planned trajectory is invalid.
[grasp_executor-1] [WARN] [1781387204.242610280] [grasp_executor_node]: Cannot wait until motion is executed (no motion is in progress).
[grasp_executor-1] [WARN] [1781387204.291291873] [grasp_executor_node]: [STEP 1/5] PRE-GRASP FAILED in 5.21s — aborting this candidate
[grasp_executor-1] [WARN] [1781387204.292507658] [grasp_executor_node]: [ATTEMPT 5/5] FAILED (PREGRASP_PLAN_FAIL) — no more candidates
[grasp_executor-1] [INFO] [1781387217.550383237] [grasp_executor_node]: ExecuteGrasp: target='box' max_attempts=5 skip_place=False
[grasp_executor-1] [INFO] [1781387217.564998203] [grasp_executor_node]: [RESET] Moving arm to home before inference (clean scene for ICGNet)...
[grasp_executor-1] [INFO] [1781387217.746457044] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781387217.756970310] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387217.767463495] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387218.096752817] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781387218.113693582] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387218.119369436] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387218.340982941] [grasp_executor_node]: [RESET] Arm at home.
[grasp_executor-1] [INFO] [1781387218.494366250] [grasp_executor_node]: [RESET] Re-added CO 'icgnet_inst_0' to scene.
[grasp_executor-1] [INFO] [1781387222.887145739] [grasp_executor_node]: [FILTER] total=1410 → kept=719 (scores=[0.00–0.32]) | rejected: width=0 workspace=0 target=0 low_prepos=691
[grasp_executor-1] [INFO] [1781387222.888154368] [grasp_executor_node]: [SELECT] 5 candidates:
[grasp_executor-1] [INFO] [1781387222.889165535] [grasp_executor_node]:   [1] score=0.32 inst=0 cls=1(box) pos=[0.647,0.162,0.084] approach=[-0.552,-0.785,-0.282] angle_from_vertical=73.6° width=0.080
[grasp_executor-1] [INFO] [1781387222.891975143] [grasp_executor_node]:   [2] score=0.32 inst=0 cls=1(box) pos=[0.641,0.153,0.087] approach=[-0.552,-0.785,-0.282] angle_from_vertical=73.6° width=0.080
[grasp_executor-1] [INFO] [1781387222.928824831] [grasp_executor_node]:   [3] score=0.31 inst=0 cls=1(box) pos=[0.639,0.151,0.087] approach=[-0.552,-0.785,-0.282] angle_from_vertical=73.6° width=0.080
[grasp_executor-1] [INFO] [1781387222.947501648] [grasp_executor_node]:   [4] score=0.31 inst=0 cls=1(box) pos=[0.649,0.165,0.088] approach=[-0.552,-0.785,-0.282] angle_from_vertical=73.6° width=0.080
[grasp_executor-1] [INFO] [1781387222.979541393] [grasp_executor_node]:   [5] score=0.31 inst=0 cls=1(box) pos=[0.646,0.161,0.087] approach=[-0.552,-0.785,-0.282] angle_from_vertical=73.6° width=0.080
[grasp_executor-1] [INFO] [1781387222.980635052] [grasp_executor_node]: [RESET] Teleport-back target = [0.611, 0.111, 0.045] (from /model_poses)
[grasp_executor-1] [INFO] [1781387222.981660285] [grasp_executor_node]: ============================================================
[grasp_executor-1] [ATTEMPT 1/5] score=0.32 inst=0 cls=1(box) pos=[0.647,0.162,0.084]
[grasp_executor-1] [INFO] [1781387223.019707172] [grasp_executor_node]: [PLAN] score=0.322  inst=0  cls=1(box)  width=0.0800m
[grasp_executor-1]        icgnet_tcp  = [0.6470, 0.1621, 0.0842]
[grasp_executor-1]        contact_pos = [0.6221, 0.1268, 0.0715]  (forward_offset=4.5cm)
[grasp_executor-1]        pre_pos     = [0.7133, 0.2563, 0.1180]
[grasp_executor-1]        lift_pos    = [0.6221, 0.1268, 0.2515]
[grasp_executor-1]        approach    = [-0.5523, -0.7846, -0.2817]  (73.6° from vertical)
[grasp_executor-1] [INFO] [1781387223.035314423] [grasp_executor_node]: [GRIPPER] pre-grasp opening=80.0mm (ICGNet=80.0mm, per finger=40.0mm) [CAPPED at max_finger_pos=40mm/side]
[grasp_executor-1] [WARN] [1781387223.036293016] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387223.041325689] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387223.481469901] [grasp_executor_node]: [STEP 1/5] PRE-GRASP → [0.713, 0.256, 0.118]
[grasp_executor-1] [WARN] [1781387223.512451416] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387223.529654791] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [WARN] [1781387228.670593585] [grasp_executor_node]: Planning failed! Error code: FAILURE
[grasp_executor-1] [WARN] [1781387228.671641979] [grasp_executor_node]: Cannot execute motion because the provided/planned trajectory is invalid.
[grasp_executor-1] [WARN] [1781387228.672518514] [grasp_executor_node]: Cannot wait until motion is executed (no motion is in progress).
[grasp_executor-1] [WARN] [1781387228.673610164] [grasp_executor_node]: [STEP 1/5] PRE-GRASP FAILED in 5.19s — aborting this candidate
[grasp_executor-1] [WARN] [1781387228.675252293] [grasp_executor_node]: [ATTEMPT 1/5] FAILED (PREGRASP_PLAN_FAIL) — resetting scene and trying next candidate
[grasp_executor-1] [INFO] [1781387228.896019820] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781387228.898318927] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387228.915034221] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387229.147079628] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781387229.182311302] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387229.183271069] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387229.389000437] [grasp_executor_node]: [RESET] Arm at home.
[grasp_executor-1] [INFO] [1781387229.428661003] [grasp_executor_node]: [RESET] Object undisturbed (<3cm from target) — skipping teleport.
[grasp_executor-1] [INFO] [1781387229.588467255] [grasp_executor_node]: [RESET] Re-added CO 'icgnet_inst_0' to scene.
[grasp_executor-1] [INFO] [1781387229.590032178] [grasp_executor_node]: ============================================================
[grasp_executor-1] [ATTEMPT 2/5] score=0.32 inst=0 cls=1(box) pos=[0.641,0.153,0.087]
[grasp_executor-1] [INFO] [1781387229.596902106] [grasp_executor_node]: [PLAN] score=0.320  inst=0  cls=1(box)  width=0.0800m
[grasp_executor-1]        icgnet_tcp  = [0.6407, 0.1532, 0.0870]
[grasp_executor-1]        contact_pos = [0.6158, 0.1179, 0.0743]  (forward_offset=4.5cm)
[grasp_executor-1]        pre_pos     = [0.7070, 0.2473, 0.1208]
[grasp_executor-1]        lift_pos    = [0.6158, 0.1179, 0.2543]
[grasp_executor-1]        approach    = [-0.5523, -0.7846, -0.2817]  (73.6° from vertical)
[grasp_executor-1] [INFO] [1781387229.622171337] [grasp_executor_node]: [GRIPPER] pre-grasp opening=80.0mm (ICGNet=80.0mm, per finger=40.0mm) [CAPPED at max_finger_pos=40mm/side]
[grasp_executor-1] [WARN] [1781387229.623236970] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387229.640458747] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387230.069151845] [grasp_executor_node]: [STEP 1/5] PRE-GRASP → [0.707, 0.247, 0.121]
[grasp_executor-1] [WARN] [1781387230.080089602] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387230.089650052] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [WARN] [1781387235.293026425] [grasp_executor_node]: Planning failed! Error code: FAILURE
[grasp_executor-1] [WARN] [1781387235.320568995] [grasp_executor_node]: Cannot execute motion because the provided/planned trajectory is invalid.
[grasp_executor-1] [WARN] [1781387235.324619950] [grasp_executor_node]: Cannot wait until motion is executed (no motion is in progress).
[grasp_executor-1] [WARN] [1781387235.325572762] [grasp_executor_node]: [STEP 1/5] PRE-GRASP FAILED in 5.26s — aborting this candidate
[grasp_executor-1] [WARN] [1781387235.326554495] [grasp_executor_node]: [ATTEMPT 2/5] FAILED (PREGRASP_PLAN_FAIL) — resetting scene and trying next candidate
[grasp_executor-1] [INFO] [1781387235.522368293] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781387235.523272130] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387235.523845696] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387235.744960786] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781387235.763362608] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387235.778183179] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387236.196793501] [grasp_executor_node]: [RESET] Arm at home.
[grasp_executor-1] [INFO] [1781387236.200313110] [grasp_executor_node]: [RESET] Object undisturbed (<3cm from target) — skipping teleport.
[grasp_executor-1] [INFO] [1781387236.383166153] [grasp_executor_node]: [RESET] Re-added CO 'icgnet_inst_0' to scene.
[grasp_executor-1] [INFO] [1781387236.403400030] [grasp_executor_node]: ============================================================
[grasp_executor-1] [ATTEMPT 3/5] score=0.31 inst=0 cls=1(box) pos=[0.639,0.151,0.087]
[grasp_executor-1] [INFO] [1781387236.432773915] [grasp_executor_node]: [PLAN] score=0.315  inst=0  cls=1(box)  width=0.0800m
[grasp_executor-1]        icgnet_tcp  = [0.6394, 0.1514, 0.0867]
[grasp_executor-1]        contact_pos = [0.6146, 0.1161, 0.0741]  (forward_offset=4.5cm)
[grasp_executor-1]        pre_pos     = [0.7057, 0.2455, 0.1205]
[grasp_executor-1]        lift_pos    = [0.6146, 0.1161, 0.2541]
[grasp_executor-1]        approach    = [-0.5523, -0.7846, -0.2817]  (73.6° from vertical)
[grasp_executor-1] [INFO] [1781387236.435068602] [grasp_executor_node]: [GRIPPER] pre-grasp opening=80.0mm (ICGNet=80.0mm, per finger=40.0mm) [CAPPED at max_finger_pos=40mm/side]
[grasp_executor-1] [WARN] [1781387236.436567747] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387236.437591384] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387236.875477177] [grasp_executor_node]: [STEP 1/5] PRE-GRASP → [0.706, 0.246, 0.121]
[grasp_executor-1] [WARN] [1781387236.876889137] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387236.877697872] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [WARN] [1781387242.118282526] [grasp_executor_node]: Planning failed! Error code: FAILURE
[grasp_executor-1] [WARN] [1781387242.124475067] [grasp_executor_node]: Cannot execute motion because the provided/planned trajectory is invalid.
[grasp_executor-1] [WARN] [1781387242.126070379] [grasp_executor_node]: Cannot wait until motion is executed (no motion is in progress).
[grasp_executor-1] [WARN] [1781387242.148654419] [grasp_executor_node]: [STEP 1/5] PRE-GRASP FAILED in 5.25s — aborting this candidate
[grasp_executor-1] [WARN] [1781387242.150358341] [grasp_executor_node]: [ATTEMPT 3/5] FAILED (PREGRASP_PLAN_FAIL) — resetting scene and trying next candidate
[grasp_executor-1] [INFO] [1781387242.330086043] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781387242.345225716] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387242.346541474] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387242.551480795] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781387242.553026324] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387242.571651814] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387242.897938503] [grasp_executor_node]: [RESET] Arm at home.
[grasp_executor-1] [INFO] [1781387242.902125334] [grasp_executor_node]: [RESET] Object undisturbed (<3cm from target) — skipping teleport.
[grasp_executor-1] [INFO] [1781387243.071911293] [grasp_executor_node]: [RESET] Re-added CO 'icgnet_inst_0' to scene.
[grasp_executor-1] [INFO] [1781387243.094041110] [grasp_executor_node]: ============================================================
[grasp_executor-1] [ATTEMPT 4/5] score=0.31 inst=0 cls=1(box) pos=[0.649,0.165,0.088]
[grasp_executor-1] [INFO] [1781387243.096809424] [grasp_executor_node]: [PLAN] score=0.315  inst=0  cls=1(box)  width=0.0800m
[grasp_executor-1]        icgnet_tcp  = [0.6492, 0.1652, 0.0875]
[grasp_executor-1]        contact_pos = [0.6243, 0.1299, 0.0748]  (forward_offset=4.5cm)
[grasp_executor-1]        pre_pos     = [0.7154, 0.2593, 0.1213]
[grasp_executor-1]        lift_pos    = [0.6243, 0.1299, 0.2548]
[grasp_executor-1]        approach    = [-0.5523, -0.7846, -0.2817]  (73.6° from vertical)
[grasp_executor-1] [INFO] [1781387243.116224858] [grasp_executor_node]: [GRIPPER] pre-grasp opening=80.0mm (ICGNet=80.0mm, per finger=40.0mm) [CAPPED at max_finger_pos=40mm/side]
[grasp_executor-1] [WARN] [1781387243.117311696] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387243.118669472] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387243.426412899] [grasp_executor_node]: [STEP 1/5] PRE-GRASP → [0.715, 0.259, 0.121]
[grasp_executor-1] [WARN] [1781387243.428185540] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387243.429439281] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [WARN] [1781387248.680261947] [grasp_executor_node]: Planning failed! Error code: FAILURE
[grasp_executor-1] [WARN] [1781387248.680962444] [grasp_executor_node]: Cannot execute motion because the provided/planned trajectory is invalid.
[grasp_executor-1] [WARN] [1781387248.681918585] [grasp_executor_node]: Cannot wait until motion is executed (no motion is in progress).
[grasp_executor-1] [WARN] [1781387248.714483189] [grasp_executor_node]: [STEP 1/5] PRE-GRASP FAILED in 5.26s — aborting this candidate
[grasp_executor-1] [WARN] [1781387248.762451711] [grasp_executor_node]: [ATTEMPT 4/5] FAILED (PREGRASP_PLAN_FAIL) — resetting scene and trying next candidate
[grasp_executor-1] [INFO] [1781387248.965428728] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781387248.966556007] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387248.967217068] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387249.317390392] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781387249.318403720] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387249.319116654] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387249.750676775] [grasp_executor_node]: [RESET] Arm at home.
[grasp_executor-1] [INFO] [1781387249.751365686] [grasp_executor_node]: [RESET] Object undisturbed (<3cm from target) — skipping teleport.
[grasp_executor-1] [INFO] [1781387249.918024008] [grasp_executor_node]: [RESET] Re-added CO 'icgnet_inst_0' to scene.
[grasp_executor-1] [INFO] [1781387249.941500436] [grasp_executor_node]: ============================================================
[grasp_executor-1] [ATTEMPT 5/5] score=0.31 inst=0 cls=1(box) pos=[0.646,0.161,0.087]
[grasp_executor-1] [INFO] [1781387249.944795641] [grasp_executor_node]: [PLAN] score=0.312  inst=0  cls=1(box)  width=0.0800m
[grasp_executor-1]        icgnet_tcp  = [0.6465, 0.1614, 0.0870]
[grasp_executor-1]        contact_pos = [0.6216, 0.1261, 0.0743]  (forward_offset=4.5cm)
[grasp_executor-1]        pre_pos     = [0.7127, 0.2555, 0.1208]
[grasp_executor-1]        lift_pos    = [0.6216, 0.1261, 0.2543]
[grasp_executor-1]        approach    = [-0.5523, -0.7846, -0.2817]  (73.6° from vertical)
[grasp_executor-1] [INFO] [1781387249.956312598] [grasp_executor_node]: [GRIPPER] pre-grasp opening=80.0mm (ICGNet=80.0mm, per finger=40.0mm) [CAPPED at max_finger_pos=40mm/side]
[grasp_executor-1] [WARN] [1781387249.962363339] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387249.963021849] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387250.282189201] [grasp_executor_node]: [STEP 1/5] PRE-GRASP → [0.713, 0.256, 0.121]
[grasp_executor-1] [WARN] [1781387250.288132157] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387250.288999540] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [WARN] [1781387255.428324199] [grasp_executor_node]: Planning failed! Error code: FAILURE
[grasp_executor-1] [WARN] [1781387255.444312734] [grasp_executor_node]: Cannot execute motion because the provided/planned trajectory is invalid.
[grasp_executor-1] [WARN] [1781387255.445585850] [grasp_executor_node]: Cannot wait until motion is executed (no motion is in progress).
[grasp_executor-1] [WARN] [1781387255.447889608] [grasp_executor_node]: [STEP 1/5] PRE-GRASP FAILED in 5.16s — aborting this candidate
[grasp_executor-1] [WARN] [1781387255.449393680] [grasp_executor_node]: [ATTEMPT 5/5] FAILED (PREGRASP_PLAN_FAIL) — no more candidates
[grasp_executor-1] [INFO] [1781387265.781913988] [grasp_executor_node]: ExecuteGrasp: target='can' max_attempts=5 skip_place=False
[grasp_executor-1] [INFO] [1781387265.797326888] [grasp_executor_node]: [RESET] Moving arm to home before inference (clean scene for ICGNet)...
[grasp_executor-1] [INFO] [1781387265.972826385] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781387265.982003662] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387265.982849055] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387266.501535263] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781387266.503207346] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387266.504051260] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387266.733277427] [grasp_executor_node]: [RESET] Arm at home.
[grasp_executor-1] [INFO] [1781387266.896114300] [grasp_executor_node]: [RESET] Re-added CO 'icgnet_inst_0' to scene.
[grasp_executor-1] [INFO] [1781387272.224240754] [grasp_executor_node]: [FILTER] total=1236 → kept=641 (scores=[0.00–0.53]) | rejected: width=0 workspace=0 target=0 low_prepos=595
[grasp_executor-1] [INFO] [1781387272.225032882] [grasp_executor_node]: [SELECT] 5 candidates:
[grasp_executor-1] [INFO] [1781387272.274548196] [grasp_executor_node]:   [1] score=0.53 inst=0 cls=2(can) pos=[0.642,0.114,0.109] approach=[-0.913,-0.309,-0.267] angle_from_vertical=74.5° width=0.080
[grasp_executor-1] [INFO] [1781387272.297356078] [grasp_executor_node]:   [2] score=0.51 inst=0 cls=2(can) pos=[0.629,0.069,0.109] approach=[-0.647,0.709,-0.279] angle_from_vertical=73.8° width=0.080
[grasp_executor-1] [INFO] [1781387272.346768876] [grasp_executor_node]:   [3] score=0.51 inst=0 cls=2(can) pos=[0.627,0.066,0.110] approach=[-0.590,0.758,-0.280] angle_from_vertical=73.7° width=0.080
[grasp_executor-1] [INFO] [1781387272.358215779] [grasp_executor_node]:   [4] score=0.50 inst=0 cls=2(can) pos=[0.627,0.066,0.110] approach=[-0.589,0.759,-0.277] angle_from_vertical=73.9° width=0.080
[grasp_executor-1] [INFO] [1781387272.386896502] [grasp_executor_node]:   [5] score=0.49 inst=0 cls=2(can) pos=[0.623,0.068,0.105] approach=[-0.540,0.793,-0.282] angle_from_vertical=73.6° width=0.080
[grasp_executor-1] [INFO] [1781387272.388358899] [grasp_executor_node]: [RESET] Teleport-back target = [0.602, 0.101, -0.003] (from /model_poses)
[grasp_executor-1] [INFO] [1781387272.424602722] [grasp_executor_node]: ============================================================
[grasp_executor-1] [ATTEMPT 1/5] score=0.53 inst=0 cls=2(can) pos=[0.642,0.114,0.109]
[grasp_executor-1] [INFO] [1781387272.503374862] [grasp_executor_node]: [PLAN] score=0.533  inst=0  cls=2(can)  width=0.0800m
[grasp_executor-1]        icgnet_tcp  = [0.6423, 0.1144, 0.1085]
[grasp_executor-1]        contact_pos = [0.6012, 0.1005, 0.0965]  (forward_offset=4.5cm)
[grasp_executor-1]        pre_pos     = [0.7518, 0.1514, 0.1406]
[grasp_executor-1]        lift_pos    = [0.6012, 0.1005, 0.2765]
[grasp_executor-1]        approach    = [-0.9129, -0.3086, -0.2671]  (74.5° from vertical)
[grasp_executor-1] [INFO] [1781387272.521808023] [grasp_executor_node]: [GRIPPER] pre-grasp opening=80.0mm (ICGNet=80.0mm, per finger=40.0mm) [CAPPED at max_finger_pos=40mm/side]
[grasp_executor-1] [WARN] [1781387272.523440472] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387272.526328910] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387272.855138132] [grasp_executor_node]: [STEP 1/5] PRE-GRASP → [0.752, 0.151, 0.141]
[grasp_executor-1] [WARN] [1781387272.876848872] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387272.879076341] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [WARN] [1781387278.102957926] [grasp_executor_node]: Planning failed! Error code: FAILURE
[grasp_executor-1] [WARN] [1781387278.104107065] [grasp_executor_node]: Cannot execute motion because the provided/planned trajectory is invalid.
[grasp_executor-1] [WARN] [1781387278.105239189] [grasp_executor_node]: Cannot wait until motion is executed (no motion is in progress).
[grasp_executor-1] [WARN] [1781387278.157125285] [grasp_executor_node]: [STEP 1/5] PRE-GRASP FAILED in 5.25s — aborting this candidate
[grasp_executor-1] [WARN] [1781387278.238885949] [grasp_executor_node]: [ATTEMPT 1/5] FAILED (PREGRASP_PLAN_FAIL) — resetting scene and trying next candidate
[grasp_executor-1] [INFO] [1781387278.454915026] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781387278.456316548] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387278.479154551] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387278.702238688] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781387278.715930182] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387278.717209634] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387279.054808592] [grasp_executor_node]: [RESET] Arm at home.
[grasp_executor-1] [INFO] [1781387279.055993516] [grasp_executor_node]: [RESET] Object undisturbed (<3cm from target) — skipping teleport.
[grasp_executor-1] [INFO] [1781387279.216486154] [grasp_executor_node]: [RESET] Re-added CO 'icgnet_inst_0' to scene.
[grasp_executor-1] [INFO] [1781387279.217420413] [grasp_executor_node]: ============================================================
[grasp_executor-1] [ATTEMPT 2/5] score=0.51 inst=0 cls=2(can) pos=[0.629,0.069,0.109]
[grasp_executor-1] [INFO] [1781387279.231978023] [grasp_executor_node]: [PLAN] score=0.509  inst=0  cls=2(can)  width=0.0800m
[grasp_executor-1]        icgnet_tcp  = [0.6293, 0.0692, 0.1093]
[grasp_executor-1]        contact_pos = [0.6002, 0.1011, 0.0967]  (forward_offset=4.5cm)
[grasp_executor-1]        pre_pos     = [0.7070, -0.0159, 0.1428]
[grasp_executor-1]        lift_pos    = [0.6002, 0.1011, 0.2767]
[grasp_executor-1]        approach    = [-0.6473, 0.7092, -0.2793]  (73.8° from vertical)
[grasp_executor-1] [INFO] [1781387279.252979135] [grasp_executor_node]: [GRIPPER] pre-grasp opening=80.0mm (ICGNet=80.0mm, per finger=40.0mm) [CAPPED at max_finger_pos=40mm/side]
[grasp_executor-1] [WARN] [1781387279.274324581] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387279.275490581] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387279.690567066] [grasp_executor_node]: [STEP 1/5] PRE-GRASP → [0.707, -0.016, 0.143]
[grasp_executor-1] [WARN] [1781387279.692292331] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387279.703917867] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [WARN] [1781387284.886435192] [grasp_executor_node]: Planning failed! Error code: FAILURE
[grasp_executor-1] [WARN] [1781387284.896954354] [grasp_executor_node]: Cannot execute motion because the provided/planned trajectory is invalid.
[grasp_executor-1] [WARN] [1781387284.897881002] [grasp_executor_node]: Cannot wait until motion is executed (no motion is in progress).
[grasp_executor-1] [WARN] [1781387284.899025012] [grasp_executor_node]: [STEP 1/5] PRE-GRASP FAILED in 5.21s — aborting this candidate
[grasp_executor-1] [WARN] [1781387284.900159507] [grasp_executor_node]: [ATTEMPT 2/5] FAILED (PREGRASP_PLAN_FAIL) — resetting scene and trying next candidate
[grasp_executor-1] [INFO] [1781387285.103395474] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781387285.117999602] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387285.118728233] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387285.345028569] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781387285.346048568] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387285.351701908] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387285.573928379] [grasp_executor_node]: [RESET] Arm at home.
[grasp_executor-1] [INFO] [1781387285.594979391] [grasp_executor_node]: [RESET] Object undisturbed (<3cm from target) — skipping teleport.
[grasp_executor-1] [INFO] [1781387285.773448243] [grasp_executor_node]: [RESET] Re-added CO 'icgnet_inst_0' to scene.
[grasp_executor-1] [INFO] [1781387285.802560609] [grasp_executor_node]: ============================================================
[grasp_executor-1] [ATTEMPT 3/5] score=0.51 inst=0 cls=2(can) pos=[0.627,0.066,0.110]
[grasp_executor-1] [INFO] [1781387285.833225070] [grasp_executor_node]: [PLAN] score=0.507  inst=0  cls=2(can)  width=0.0800m
[grasp_executor-1]        icgnet_tcp  = [0.6272, 0.0663, 0.1100]
[grasp_executor-1]        contact_pos = [0.6007, 0.1004, 0.0974]  (forward_offset=4.5cm)
[grasp_executor-1]        pre_pos     = [0.6979, -0.0246, 0.1436]
[grasp_executor-1]        lift_pos    = [0.6007, 0.1004, 0.2774]
[grasp_executor-1]        approach    = [-0.5896, 0.7576, -0.2802]  (73.7° from vertical)
[grasp_executor-1] [INFO] [1781387285.839008553] [grasp_executor_node]: [GRIPPER] pre-grasp opening=80.0mm (ICGNet=80.0mm, per finger=40.0mm) [CAPPED at max_finger_pos=40mm/side]
[grasp_executor-1] [WARN] [1781387285.840231431] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387285.845254250] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387286.269574264] [grasp_executor_node]: [STEP 1/5] PRE-GRASP → [0.698, -0.025, 0.144]
[grasp_executor-1] [WARN] [1781387286.271203641] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387286.272428105] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [WARN] [1781387291.472444385] [grasp_executor_node]: Planning failed! Error code: FAILURE
[grasp_executor-1] [WARN] [1781387291.480921576] [grasp_executor_node]: Cannot execute motion because the provided/planned trajectory is invalid.
[grasp_executor-1] [WARN] [1781387291.509541664] [grasp_executor_node]: Cannot wait until motion is executed (no motion is in progress).
[grasp_executor-1] [WARN] [1781387291.531167234] [grasp_executor_node]: [STEP 1/5] PRE-GRASP FAILED in 5.24s — aborting this candidate
[grasp_executor-1] [WARN] [1781387291.541683273] [grasp_executor_node]: [ATTEMPT 3/5] FAILED (PREGRASP_PLAN_FAIL) — resetting scene and trying next candidate
[grasp_executor-1] [INFO] [1781387290.030555337] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781387290.043540596] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387290.046739047] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387290.266756772] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781387290.267985726] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387290.268968678] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387290.493378181] [grasp_executor_node]: [RESET] Arm at home.
[grasp_executor-1] [INFO] [1781387290.517399815] [grasp_executor_node]: [RESET] Object undisturbed (<3cm from target) — skipping teleport.
[grasp_executor-1] [INFO] [1781387290.690301829] [grasp_executor_node]: [RESET] Re-added CO 'icgnet_inst_0' to scene.
[grasp_executor-1] [INFO] [1781387290.691767905] [grasp_executor_node]: ============================================================
[grasp_executor-1] [ATTEMPT 4/5] score=0.50 inst=0 cls=2(can) pos=[0.627,0.066,0.110]
[grasp_executor-1] [INFO] [1781387290.694704074] [grasp_executor_node]: [PLAN] score=0.502  inst=0  cls=2(can)  width=0.0800m
[grasp_executor-1]        icgnet_tcp  = [0.6272, 0.0664, 0.1103]
[grasp_executor-1]        contact_pos = [0.6007, 0.1005, 0.0979]  (forward_offset=4.5cm)
[grasp_executor-1]        pre_pos     = [0.6979, -0.0247, 0.1436]
[grasp_executor-1]        lift_pos    = [0.6007, 0.1005, 0.2779]
[grasp_executor-1]        approach    = [-0.5895, 0.7587, -0.2772]  (73.9° from vertical)
[grasp_executor-1] [INFO] [1781387290.709701645] [grasp_executor_node]: [GRIPPER] pre-grasp opening=80.0mm (ICGNet=80.0mm, per finger=40.0mm) [CAPPED at max_finger_pos=40mm/side]
[grasp_executor-1] [WARN] [1781387290.727259669] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387290.734919460] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387291.217220244] [grasp_executor_node]: [STEP 1/5] PRE-GRASP → [0.698, -0.025, 0.144]
[grasp_executor-1] [WARN] [1781387291.218689915] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387291.249356779] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [WARN] [1781387296.570705480] [grasp_executor_node]: Planning failed! Error code: FAILURE
[grasp_executor-1] [WARN] [1781387296.590839772] [grasp_executor_node]: Cannot execute motion because the provided/planned trajectory is invalid.
[grasp_executor-1] [WARN] [1781387296.621257308] [grasp_executor_node]: Cannot wait until motion is executed (no motion is in progress).
[grasp_executor-1] [WARN] [1781387296.642615343] [grasp_executor_node]: [STEP 1/5] PRE-GRASP FAILED in 5.40s — aborting this candidate
[grasp_executor-1] [WARN] [1781387296.644159320] [grasp_executor_node]: [ATTEMPT 4/5] FAILED (PREGRASP_PLAN_FAIL) — resetting scene and trying next candidate
[grasp_executor-1] [INFO] [1781387296.892030571] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781387296.893095623] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387296.894116589] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387297.214981996] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781387297.216764590] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387297.244686008] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387297.459616118] [grasp_executor_node]: [RESET] Arm at home.
[grasp_executor-1] [INFO] [1781387297.465537552] [grasp_executor_node]: [RESET] Object undisturbed (<3cm from target) — skipping teleport.
[grasp_executor-1] [INFO] [1781387297.641888318] [grasp_executor_node]: [RESET] Re-added CO 'icgnet_inst_0' to scene.
[grasp_executor-1] [INFO] [1781387297.647602112] [grasp_executor_node]: ============================================================
[grasp_executor-1] [ATTEMPT 5/5] score=0.49 inst=0 cls=2(can) pos=[0.623,0.068,0.105]
[grasp_executor-1] [INFO] [1781387297.672861745] [grasp_executor_node]: [PLAN] score=0.490  inst=0  cls=2(can)  width=0.0800m
[grasp_executor-1]        icgnet_tcp  = [0.6225, 0.0677, 0.1045]
[grasp_executor-1]        contact_pos = [0.5983, 0.1034, 0.0918]  (forward_offset=4.5cm)
[grasp_executor-1]        pre_pos     = [0.6873, -0.0275, 0.1383]
[grasp_executor-1]        lift_pos    = [0.5983, 0.1034, 0.2718]
[grasp_executor-1]        approach    = [-0.5395, 0.7934, -0.2817]  (73.6° from vertical)
[grasp_executor-1] [INFO] [1781387297.687204853] [grasp_executor_node]: [GRIPPER] pre-grasp opening=80.0mm (ICGNet=80.0mm, per finger=40.0mm) [CAPPED at max_finger_pos=40mm/side]
[grasp_executor-1] [WARN] [1781387297.688381866] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387297.761439562] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387298.334396573] [grasp_executor_node]: [STEP 1/5] PRE-GRASP → [0.687, -0.028, 0.138]
[grasp_executor-1] [WARN] [1781387298.335856923] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387298.336695940] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [WARN] [1781387303.614223078] [grasp_executor_node]: Planning failed! Error code: FAILURE
[grasp_executor-1] [WARN] [1781387303.614802917] [grasp_executor_node]: Cannot execute motion because the provided/planned trajectory is invalid.
[grasp_executor-1] [WARN] [1781387303.615892219] [grasp_executor_node]: Cannot wait until motion is executed (no motion is in progress).
[grasp_executor-1] [WARN] [1781387303.617312554] [grasp_executor_node]: [STEP 1/5] PRE-GRASP FAILED in 5.28s — aborting this candidate
[grasp_executor-1] [WARN] [1781387303.618340051] [grasp_executor_node]: [ATTEMPT 5/5] FAILED (PREGRASP_PLAN_FAIL) — no more candidates
[grasp_executor-1] [INFO] [1781387315.914178880] [grasp_executor_node]: ExecuteGrasp: target='can' max_attempts=5 skip_place=False
[grasp_executor-1] [INFO] [1781387315.919479594] [grasp_executor_node]: [RESET] Moving arm to home before inference (clean scene for ICGNet)...
[grasp_executor-1] [INFO] [1781387316.105874101] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781387316.120838979] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387316.126752188] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387316.336681647] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781387316.337650868] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387316.338727075] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387316.764803122] [grasp_executor_node]: [RESET] Arm at home.
[grasp_executor-1] [INFO] [1781387316.925965616] [grasp_executor_node]: [RESET] Re-added CO 'icgnet_inst_0' to scene.
[grasp_executor-1] [INFO] [1781387321.376742526] [grasp_executor_node]: [FILTER] total=1230 → kept=974 (scores=[0.00–0.77]) | rejected: width=0 workspace=0 target=0 low_prepos=256
[grasp_executor-1] [INFO] [1781387321.387079476] [grasp_executor_node]: [SELECT] 5 candidates:
[grasp_executor-1] [INFO] [1781387321.437751025] [grasp_executor_node]:   [1] score=0.77 inst=0 cls=2(can) pos=[0.537,-0.069,0.109] approach=[-0.680,-0.681,-0.271] angle_from_vertical=74.3° width=0.080
[grasp_executor-1] [INFO] [1781387321.463719431] [grasp_executor_node]:   [2] score=0.77 inst=0 cls=2(can) pos=[0.531,-0.070,0.120] approach=[-0.545,-0.658,-0.520] angle_from_vertical=58.6° width=0.080
[grasp_executor-1] [INFO] [1781387321.487241682] [grasp_executor_node]:   [3] score=0.77 inst=0 cls=2(can) pos=[0.501,-0.096,0.143] approach=[0.126,-0.092,-0.988] angle_from_vertical=9.0° width=0.080
[grasp_executor-1] [INFO] [1781387321.490124182] [grasp_executor_node]:   [4] score=0.77 inst=0 cls=2(can) pos=[0.523,-0.074,0.130] approach=[-0.365,-0.581,-0.727] angle_from_vertical=43.3° width=0.080
[grasp_executor-1] [INFO] [1781387321.491400971] [grasp_executor_node]:   [5] score=0.76 inst=0 cls=2(can) pos=[0.501,-0.096,0.144] approach=[0.125,-0.087,-0.988] angle_from_vertical=8.7° width=0.080
[grasp_executor-1] [INFO] [1781387321.492293538] [grasp_executor_node]: [RESET] Teleport-back target = [0.506, -0.099, -0.003] (from /model_poses)
[grasp_executor-1] [INFO] [1781387321.493183674] [grasp_executor_node]: ============================================================
[grasp_executor-1] [ATTEMPT 1/5] score=0.77 inst=0 cls=2(can) pos=[0.537,-0.069,0.109]
[grasp_executor-1] [INFO] [1781387321.496687757] [grasp_executor_node]: [PLAN] score=0.773  inst=0  cls=2(can)  width=0.0800m
[grasp_executor-1]        icgnet_tcp  = [0.5367, -0.0692, 0.1092]
[grasp_executor-1]        contact_pos = [0.5061, -0.0999, 0.0970]  (forward_offset=4.5cm)
[grasp_executor-1]        pre_pos     = [0.6184, 0.0125, 0.1418]
[grasp_executor-1]        lift_pos    = [0.5061, -0.0999, 0.2770]
[grasp_executor-1]        approach    = [-0.6801, -0.6811, -0.2712]  (74.3° from vertical)
[grasp_executor-1] [INFO] [1781387321.498229080] [grasp_executor_node]: [GRIPPER] pre-grasp opening=80.0mm (ICGNet=80.0mm, per finger=40.0mm) [CAPPED at max_finger_pos=40mm/side]
[grasp_executor-1] [WARN] [1781387321.499403099] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387321.500173126] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387321.833575100] [grasp_executor_node]: [STEP 1/5] PRE-GRASP → [0.618, 0.013, 0.142]
[grasp_executor-1] [WARN] [1781387321.863300705] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387321.867158030] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [WARN] [1781387327.112957105] [grasp_executor_node]: Planning failed! Error code: FAILURE
[grasp_executor-1] [WARN] [1781387327.113955296] [grasp_executor_node]: Cannot execute motion because the provided/planned trajectory is invalid.
[grasp_executor-1] [WARN] [1781387327.137257313] [grasp_executor_node]: Cannot wait until motion is executed (no motion is in progress).
[grasp_executor-1] [WARN] [1781387327.176885752] [grasp_executor_node]: [STEP 1/5] PRE-GRASP FAILED in 5.30s — aborting this candidate
[grasp_executor-1] [WARN] [1781387327.187141103] [grasp_executor_node]: [ATTEMPT 1/5] FAILED (PREGRASP_PLAN_FAIL) — resetting scene and trying next candidate
[grasp_executor-1] [INFO] [1781387327.373780375] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781387327.375054129] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387327.376221384] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387327.588413760] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781387327.602610105] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387327.646379627] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387327.980472343] [grasp_executor_node]: [RESET] Arm at home.
[grasp_executor-1] [INFO] [1781387328.001891885] [grasp_executor_node]: [RESET] Object undisturbed (<3cm from target) — skipping teleport.
[grasp_executor-1] [INFO] [1781387328.185753501] [grasp_executor_node]: [RESET] Re-added CO 'icgnet_inst_0' to scene.
[grasp_executor-1] [INFO] [1781387328.220659995] [grasp_executor_node]: ============================================================
[grasp_executor-1] [ATTEMPT 2/5] score=0.77 inst=0 cls=2(can) pos=[0.531,-0.070,0.120]
[grasp_executor-1] [INFO] [1781387328.231846312] [grasp_executor_node]: [PLAN] score=0.772  inst=0  cls=2(can)  width=0.0800m
[grasp_executor-1]        icgnet_tcp  = [0.5306, -0.0703, 0.1204]
[grasp_executor-1]        contact_pos = [0.5061, -0.0999, 0.0970]  (forward_offset=4.5cm)
[grasp_executor-1]        pre_pos     = [0.5960, 0.0087, 0.1829]
[grasp_executor-1]        lift_pos    = [0.5061, -0.0999, 0.2770]
[grasp_executor-1]        approach    = [-0.5446, -0.6578, -0.5204]  (58.6° from vertical)
[grasp_executor-1] [INFO] [1781387328.233532936] [grasp_executor_node]: [GRIPPER] pre-grasp opening=80.0mm (ICGNet=80.0mm, per finger=40.0mm) [CAPPED at max_finger_pos=40mm/side]
[grasp_executor-1] [WARN] [1781387328.234940607] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387328.248302533] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387328.554409817] [grasp_executor_node]: [STEP 1/5] PRE-GRASP → [0.596, 0.009, 0.183]
[grasp_executor-1] [WARN] [1781387328.556274186] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387328.557749187] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387336.321362406] [grasp_executor_node]: [STEP 1/5] Pre-grasp reached in 7.77s
[grasp_executor-1] [INFO] [1781387337.147448231] [grasp_executor_node]: [CO] Removed 'icgnet_inst_0' from scene for approach
[grasp_executor-1] [INFO] [1781387337.148247215] [grasp_executor_node]: [STEP 2/5] CARTESIAN APPROACH → [0.506, -0.100, 0.097]  (vel=0.08)
[grasp_executor-1] [WARN] [1781387337.149203434] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387337.149885419] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387339.624023880] [grasp_executor_node]: [STEP 2/5] Contact surface reached in 2.45s
[grasp_executor-1] [INFO] [1781387339.649708162] [grasp_executor_node]: [STEP 3/5] CLOSING GRIPPER
[grasp_executor-1] [WARN] [1781387339.678066384] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387339.683614380] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387340.522074190] [grasp_executor_node]: [STEP 3/5] Object confirmed between fingers (gap=36.2mm, range [5–40mm])
[grasp_executor-1] [INFO] [1781387340.707565796] [grasp_executor_node]: [STEP 3b] Attached 'icgnet_inst_0' to panda_hand_tcp
[grasp_executor-1] [INFO] [1781387340.718211895] [grasp_executor_node]: [STEP 4/5] CARTESIAN LIFT → [0.506, -0.100, 0.277]
[grasp_executor-1] [WARN] [1781387340.719988275] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387340.720917599] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387342.868851328] [grasp_executor_node]: [STEP 4/5] Object lifted in 2.15s
[grasp_executor-1] [INFO] [1781387342.925299996] [grasp_executor_node]: [STEP 5/7] TRANSFER → ['0.450', '-0.500', '0.350']  (joint-space, orient=target, vel=0.25)
[grasp_executor-1] [WARN] [1781387342.930621934] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387342.931397095] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387350.923933284] [grasp_executor_node]: [STEP 5/7] Transfer done in 7.98s
[grasp_executor-1] [INFO] [1781387350.959889668] [grasp_executor_node]: [STEP 6/7] LOWER → ['0.450', '-0.500', '0.260']
[grasp_executor-1] [WARN] [1781387350.995251313] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387351.012781333] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387352.261590804] [grasp_executor_node]: [STEP 6/7] Release position reached in 1.28s
[grasp_executor-1] [INFO] [1781387352.305199485] [grasp_executor_node]: [STEP 7/7] RELEASING OBJECT
[grasp_executor-1] [WARN] [1781387352.339588745] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387352.368720470] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387351.247819293] [grasp_executor_node]: [STEP 7/7] Finger gap after open: 36.0mm (~0=gripper stuck/not open, ~40=fully open)
[grasp_executor-1] [INFO] [1781387351.249775764] [grasp_executor_node]: [STEP 7/7] Detached 'icgnet_inst_0'
[grasp_executor-1] [INFO] [1781387351.250763673] [grasp_executor_node]: [RETRACT] Cartesian retract → z=0.360
[grasp_executor-1] [WARN] [1781387351.252285655] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387351.263667799] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387352.795461661] [grasp_executor_node]: [HOME] Returning to home (object released)
[grasp_executor-1] [WARN] [1781387352.796824013] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387352.798194928] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387361.365950839] [grasp_executor_node]: [HOME] Reached in 8.57s
[grasp_executor-1] [INFO] [1781387361.367736425] [grasp_executor_node]: [PLACE] SUCCESS: object in bin (pose=[0.459, -0.511, -0.000], bin=[0.450, -0.500] ±0.150)
[grasp_executor-1] [INFO] [1781387361.397383282] [grasp_executor_node]: [SUCCESS] Grasp completed on attempt 2/5
[grasp_executor-1] [INFO] [1781387376.392884322] [grasp_executor_node]: ExecuteGrasp: target='can' max_attempts=5 skip_place=False
[grasp_executor-1] [INFO] [1781387376.398601919] [grasp_executor_node]: [RESET] Moving arm to home before inference (clean scene for ICGNet)...
[grasp_executor-1] [INFO] [1781387376.399149993] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781387376.399757413] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387376.400947922] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387376.921283910] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781387376.930895045] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387376.932097402] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387377.145477400] [grasp_executor_node]: [RESET] Arm at home.
[grasp_executor-1] [INFO] [1781387381.546424340] [grasp_executor_node]: [FILTER] total=1416 → kept=897 (scores=[0.00–0.78]) | rejected: width=0 workspace=0 target=0 low_prepos=519
[grasp_executor-1] [INFO] [1781387381.547263007] [grasp_executor_node]: [SELECT] 5 candidates:
[grasp_executor-1] [INFO] [1781387381.557712280] [grasp_executor_node]:   [1] score=0.78 inst=0 cls=2(can) pos=[0.567,-0.243,0.142] approach=[0.241,-0.040,-0.970] angle_from_vertical=14.1° width=0.080
[grasp_executor-1] [INFO] [1781387381.598900365] [grasp_executor_node]:   [2] score=0.78 inst=0 cls=2(can) pos=[0.569,-0.242,0.142] approach=[0.192,-0.055,-0.980] angle_from_vertical=11.5° width=0.080
[grasp_executor-1] [INFO] [1781387381.628181454] [grasp_executor_node]:   [3] score=0.77 inst=0 cls=2(can) pos=[0.568,-0.242,0.141] approach=[0.225,-0.045,-0.973] angle_from_vertical=13.3° width=0.080
[grasp_executor-1] [INFO] [1781387381.639904807] [grasp_executor_node]:   [4] score=0.77 inst=0 cls=2(can) pos=[0.562,-0.231,0.138] approach=[0.347,-0.292,-0.891] angle_from_vertical=27.0° width=0.080
[grasp_executor-1] [INFO] [1781387381.642650256] [grasp_executor_node]:   [5] score=0.77 inst=0 cls=2(can) pos=[0.564,-0.231,0.139] approach=[0.317,-0.297,-0.901] angle_from_vertical=25.8° width=0.080
[grasp_executor-1] [INFO] [1781387381.643688328] [grasp_executor_node]: [RESET] Teleport-back target = [0.579, -0.244, -0.003] (from /model_poses)
[grasp_executor-1] [INFO] [1781387381.644685885] [grasp_executor_node]: ============================================================
[grasp_executor-1] [ATTEMPT 1/5] score=0.78 inst=0 cls=2(can) pos=[0.567,-0.243,0.142]
[grasp_executor-1] [INFO] [1781387381.686385443] [grasp_executor_node]: [PLAN] score=0.778  inst=0  cls=2(can)  width=0.0800m
[grasp_executor-1]        icgnet_tcp  = [0.5670, -0.2426, 0.1419]
[grasp_executor-1]        contact_pos = [0.5778, -0.2444, 0.0982]  (forward_offset=4.5cm)
[grasp_executor-1]        pre_pos     = [0.5381, -0.2379, 0.2582]
[grasp_executor-1]        lift_pos    = [0.5778, -0.2444, 0.2782]
[grasp_executor-1]        approach    = [0.2411, -0.0399, -0.9697]  (14.1° from vertical)
[grasp_executor-1] [INFO] [1781387381.720327226] [grasp_executor_node]: [GRIPPER] pre-grasp opening=80.0mm (ICGNet=80.0mm, per finger=40.0mm) [CAPPED at max_finger_pos=40mm/side]
[grasp_executor-1] [WARN] [1781387381.721572215] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387381.722738076] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387382.157084898] [grasp_executor_node]: [STEP 1/5] PRE-GRASP → [0.538, -0.238, 0.258]
[grasp_executor-1] [WARN] [1781387382.164356799] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387382.165711276] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387385.585021810] [grasp_executor_node]: [STEP 1/5] Pre-grasp reached in 3.42s
[grasp_executor-1] [INFO] [1781387386.411072131] [grasp_executor_node]: [CO] Removed 'icgnet_inst_0' from scene for approach
[grasp_executor-1] [INFO] [1781387386.430838485] [grasp_executor_node]: [STEP 2/5] CARTESIAN APPROACH → [0.578, -0.244, 0.098]  (vel=0.08)
[grasp_executor-1] [WARN] [1781387386.434030661] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387386.465009877] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387388.210842410] [grasp_executor_node]: [STEP 2/5] Contact surface reached in 1.77s
[grasp_executor-1] [INFO] [1781387388.222293049] [grasp_executor_node]: [STEP 3/5] CLOSING GRIPPER
[grasp_executor-1] [WARN] [1781387388.248478085] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387388.264795578] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387389.123144978] [grasp_executor_node]: [STEP 3/5] Object confirmed between fingers (gap=35.4mm, range [5–40mm])
[grasp_executor-1] [INFO] [1781387389.300290909] [grasp_executor_node]: [STEP 3b] Attached 'icgnet_inst_0' to panda_hand_tcp
[grasp_executor-1] [INFO] [1781387389.308591624] [grasp_executor_node]: [STEP 4/5] CARTESIAN LIFT → [0.578, -0.244, 0.278]
[grasp_executor-1] [WARN] [1781387389.310067398] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387389.321522156] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387391.591538685] [grasp_executor_node]: [STEP 4/5] Object lifted in 2.26s
[grasp_executor-1] [INFO] [1781387391.626551172] [grasp_executor_node]: [STEP 5/7] TRANSFER → ['0.450', '-0.500', '0.350']  (joint-space, orient=target, vel=0.25)
[grasp_executor-1] [WARN] [1781387391.669425183] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387391.680294159] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387408.270233079] [grasp_executor_node]: [STEP 5/7] Transfer done in 16.63s
[grasp_executor-1] [INFO] [1781387408.286438777] [grasp_executor_node]: [STEP 6/7] LOWER → ['0.450', '-0.500', '0.260']
[grasp_executor-1] [WARN] [1781387408.306554710] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387408.307463086] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387409.831016955] [grasp_executor_node]: [STEP 6/7] Release position reached in 1.54s
[grasp_executor-1] [INFO] [1781387409.871607649] [grasp_executor_node]: [STEP 7/7] RELEASING OBJECT
[grasp_executor-1] [WARN] [1781387409.875964621] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387409.887068007] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387410.662736484] [grasp_executor_node]: [STEP 7/7] Finger gap after open: 34.9mm (~0=gripper stuck/not open, ~40=fully open)
[grasp_executor-1] [INFO] [1781387410.694560190] [grasp_executor_node]: [STEP 7/7] Detached 'icgnet_inst_0'
[grasp_executor-1] [INFO] [1781387410.701931065] [grasp_executor_node]: [RETRACT] Cartesian retract → z=0.360
[grasp_executor-1] [WARN] [1781387410.754999068] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387410.761807822] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387412.043193568] [grasp_executor_node]: [HOME] Returning to home (object released)
[grasp_executor-1] [WARN] [1781387412.044362929] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387412.045739494] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387418.647404163] [grasp_executor_node]: [HOME] Reached in 6.58s
[grasp_executor-1] [WARN] [1781387418.678779875] [grasp_executor_node]: [PLACE] FAILURE: object NOT in bin after release — pose=[0.323, 0.013, 0.439], bin=[0.450, -0.500] ±0.150
[grasp_executor-1] [WARN] [1781387418.721098918] [grasp_executor_node]: [ATTEMPT 1/5] FAILED (PLACE_ROLLOUT) — resetting scene and trying next candidate
[grasp_executor-1] [INFO] [1781387418.726938039] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781387418.737010977] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387418.738105311] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387419.165474821] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781387419.203088179] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387419.207925861] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387419.549815107] [grasp_executor_node]: [RESET] Arm at home.
[grasp_executor-1] [INFO] [1781387419.623086607] [grasp_executor_node]: [RESET] Object "target_obj" reset to [0.579, -0.244, -0.003]
[grasp_executor-1] [INFO] [1781387419.624245512] [grasp_executor_node]: ============================================================
[grasp_executor-1] [ATTEMPT 2/5] score=0.78 inst=0 cls=2(can) pos=[0.569,-0.242,0.142]
[grasp_executor-1] [INFO] [1781387419.656324429] [grasp_executor_node]: [PLAN] score=0.778  inst=0  cls=2(can)  width=0.0800m
[grasp_executor-1]        icgnet_tcp  = [0.5692, -0.2421, 0.1422]
[grasp_executor-1]        contact_pos = [0.5779, -0.2445, 0.0981]  (forward_offset=4.5cm)
[grasp_executor-1]        pre_pos     = [0.5462, -0.2355, 0.2598]
[grasp_executor-1]        lift_pos    = [0.5779, -0.2445, 0.2781]
[grasp_executor-1]        approach    = [0.1918, -0.0545, -0.9799]  (11.5° from vertical)
[grasp_executor-1] [INFO] [1781387419.698506298] [grasp_executor_node]: [GRIPPER] pre-grasp opening=80.0mm (ICGNet=80.0mm, per finger=40.0mm) [CAPPED at max_finger_pos=40mm/side]
[grasp_executor-1] [WARN] [1781387419.714018986] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387419.753952584] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387420.288150258] [grasp_executor_node]: [STEP 1/5] PRE-GRASP → [0.546, -0.236, 0.260]
[grasp_executor-1] [WARN] [1781387420.319010349] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387420.320194934] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387428.005367244] [grasp_executor_node]: [STEP 1/5] Pre-grasp reached in 7.71s
[grasp_executor-1] [WARN] [1781387428.831655293] [grasp_executor_node]: [CO] 'icgnet_inst_0' not found in planning scene
[grasp_executor-1] [INFO] [1781387428.865616425] [grasp_executor_node]: [CO] Removed 'icgnet_inst_0' from scene for approach
[grasp_executor-1] [INFO] [1781387428.902066643] [grasp_executor_node]: [STEP 2/5] CARTESIAN APPROACH → [0.578, -0.245, 0.098]  (vel=0.08)
[grasp_executor-1] [WARN] [1781387428.903888398] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387428.904894525] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387430.770198844] [grasp_executor_node]: [STEP 2/5] Contact surface reached in 1.84s
[grasp_executor-1] [INFO] [1781387430.771088725] [grasp_executor_node]: [STEP 3/5] CLOSING GRIPPER
[grasp_executor-1] [WARN] [1781387430.772162548] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387430.773058246] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387431.624505486] [grasp_executor_node]: [STEP 3/5] Object confirmed between fingers (gap=34.4mm, range [5–40mm])
[grasp_executor-1] [INFO] [1781387431.633821148] [grasp_executor_node]: [STEP 3b] Attached 'icgnet_inst_0' to panda_hand_tcp
[grasp_executor-1] [INFO] [1781387431.635052926] [grasp_executor_node]: [STEP 4/5] CARTESIAN LIFT → [0.578, -0.245, 0.278]
[grasp_executor-1] [WARN] [1781387431.640579528] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387431.657444854] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387433.797533403] [grasp_executor_node]: [STEP 4/5] Object lifted in 2.16s
[grasp_executor-1] [INFO] [1781387433.836603626] [grasp_executor_node]: [STEP 5/7] TRANSFER → ['0.450', '-0.500', '0.350']  (joint-space, orient=target, vel=0.25)
[grasp_executor-1] [WARN] [1781387433.866183110] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387433.905689808] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387439.563165419] [grasp_executor_node]: [STEP 5/7] Transfer done in 5.71s
[grasp_executor-1] [INFO] [1781387439.598293643] [grasp_executor_node]: [STEP 6/7] LOWER → ['0.450', '-0.500', '0.260']
[grasp_executor-1] [WARN] [1781387439.637749515] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387439.640156769] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387440.898813307] [grasp_executor_node]: [STEP 6/7] Release position reached in 1.27s
[grasp_executor-1] [INFO] [1781387440.945050262] [grasp_executor_node]: [STEP 7/7] RELEASING OBJECT
[grasp_executor-1] [WARN] [1781387440.970307253] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387440.987582440] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387441.866662961] [grasp_executor_node]: [STEP 7/7] Finger gap after open: 35.1mm (~0=gripper stuck/not open, ~40=fully open)
[grasp_executor-1] [INFO] [1781387441.889829100] [grasp_executor_node]: [STEP 7/7] Detached 'icgnet_inst_0'
[grasp_executor-1] [INFO] [1781387441.890701900] [grasp_executor_node]: [RETRACT] Cartesian retract → z=0.360
[grasp_executor-1] [WARN] [1781387441.892943873] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387441.919058129] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387443.038072985] [grasp_executor_node]: [HOME] Returning to home (object released)
[grasp_executor-1] [WARN] [1781387443.040098845] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387443.041031354] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387448.671658982] [grasp_executor_node]: [HOME] Reached in 5.61s
[grasp_executor-1] [WARN] [1781387448.688958243] [grasp_executor_node]: [PLACE] FAILURE: object NOT in bin after release — pose=[0.336, 0.010, 0.439], bin=[0.450, -0.500] ±0.150
[grasp_executor-1] [WARN] [1781387448.689784578] [grasp_executor_node]: [ATTEMPT 2/5] FAILED (PLACE_ROLLOUT) — resetting scene and trying next candidate
[grasp_executor-1] [INFO] [1781387448.690817153] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781387448.691925639] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387448.706278148] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387449.123069654] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781387449.151509355] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387449.157921845] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387449.372105122] [grasp_executor_node]: [RESET] Arm at home.
[grasp_executor-1] [INFO] [1781387449.413709018] [grasp_executor_node]: [RESET] Object "target_obj" reset to [0.579, -0.244, -0.003]
[grasp_executor-1] [INFO] [1781387449.439082968] [grasp_executor_node]: ============================================================
[grasp_executor-1] [ATTEMPT 3/5] score=0.77 inst=0 cls=2(can) pos=[0.568,-0.242,0.141]
[grasp_executor-1] [INFO] [1781387449.456184034] [grasp_executor_node]: [PLAN] score=0.773  inst=0  cls=2(can)  width=0.0800m
[grasp_executor-1]        icgnet_tcp  = [0.5679, -0.2424, 0.1415]
[grasp_executor-1]        contact_pos = [0.5780, -0.2444, 0.0977]  (forward_offset=4.5cm)
[grasp_executor-1]        pre_pos     = [0.5409, -0.2369, 0.2583]
[grasp_executor-1]        lift_pos    = [0.5780, -0.2444, 0.2777]
[grasp_executor-1]        approach    = [0.2251, -0.0455, -0.9733]  (13.3° from vertical)
[grasp_executor-1] [INFO] [1781387449.457332011] [grasp_executor_node]: [GRIPPER] pre-grasp opening=80.0mm (ICGNet=80.0mm, per finger=40.0mm) [CAPPED at max_finger_pos=40mm/side]
[grasp_executor-1] [WARN] [1781387449.458267077] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387449.459179169] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387449.996936332] [grasp_executor_node]: [STEP 1/5] PRE-GRASP → [0.541, -0.237, 0.258]
[grasp_executor-1] [WARN] [1781387450.014898778] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387450.038073542] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387456.076288352] [grasp_executor_node]: [STEP 1/5] Pre-grasp reached in 6.07s
[grasp_executor-1] [WARN] [1781387456.904169553] [grasp_executor_node]: [CO] 'icgnet_inst_0' not found in planning scene
[grasp_executor-1] [INFO] [1781387456.927149010] [grasp_executor_node]: [CO] Removed 'icgnet_inst_0' from scene for approach
[grasp_executor-1] [INFO] [1781387456.965393083] [grasp_executor_node]: [STEP 2/5] CARTESIAN APPROACH → [0.578, -0.244, 0.098]  (vel=0.08)
[grasp_executor-1] [WARN] [1781387456.973233791] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387456.975253590] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387459.127730812] [grasp_executor_node]: [STEP 2/5] Contact surface reached in 2.16s
[grasp_executor-1] [INFO] [1781387459.128840409] [grasp_executor_node]: [STEP 3/5] CLOSING GRIPPER
[grasp_executor-1] [WARN] [1781387459.130033388] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387459.131121054] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387459.968150577] [grasp_executor_node]: [STEP 3/5] Object confirmed between fingers (gap=35.4mm, range [5–40mm])
[grasp_executor-1] [INFO] [1781387460.011096426] [grasp_executor_node]: [STEP 3b] Attached 'icgnet_inst_0' to panda_hand_tcp
[grasp_executor-1] [INFO] [1781387460.035468150] [grasp_executor_node]: [STEP 4/5] CARTESIAN LIFT → [0.578, -0.244, 0.278]
[grasp_executor-1] [WARN] [1781387460.072939965] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387460.094220116] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387462.354520345] [grasp_executor_node]: [STEP 4/5] Object lifted in 2.32s
[grasp_executor-1] [INFO] [1781387462.936261476] [grasp_executor_node]: [STEP 5/7] TRANSFER → ['0.450', '-0.500', '0.350']  (joint-space, orient=target, vel=0.25)
[grasp_executor-1] [WARN] [1781387462.937754288] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387462.959971341] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387475.660904954] [grasp_executor_node]: [STEP 5/7] Transfer done in 12.70s
[grasp_executor-1] [INFO] [1781387475.689138247] [grasp_executor_node]: [STEP 6/7] LOWER → ['0.450', '-0.500', '0.260']
[grasp_executor-1] [WARN] [1781387475.718273409] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387475.731187492] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387477.054216474] [grasp_executor_node]: [STEP 6/7] Release position reached in 1.36s
[grasp_executor-1] [INFO] [1781387477.102970748] [grasp_executor_node]: [STEP 7/7] RELEASING OBJECT
[grasp_executor-1] [WARN] [1781387477.114531813] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387477.137194446] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387477.896811841] [grasp_executor_node]: [STEP 7/7] Finger gap after open: 35.1mm (~0=gripper stuck/not open, ~40=fully open)
[grasp_executor-1] [INFO] [1781387477.899056367] [grasp_executor_node]: [STEP 7/7] Detached 'icgnet_inst_0'
[grasp_executor-1] [INFO] [1781387477.906863094] [grasp_executor_node]: [RETRACT] Cartesian retract → z=0.360
[grasp_executor-1] [WARN] [1781387477.951035170] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387477.985283926] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387479.223162201] [grasp_executor_node]: [HOME] Returning to home (object released)
[grasp_executor-1] [WARN] [1781387479.274825191] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387479.301707687] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387484.632214250] [grasp_executor_node]: [HOME] Reached in 5.41s
[grasp_executor-1] [WARN] [1781387484.633456267] [grasp_executor_node]: [PLACE] FAILURE: object NOT in bin after release — pose=[0.347, 0.006, 0.444], bin=[0.450, -0.500] ±0.150
[grasp_executor-1] [WARN] [1781387484.635001992] [grasp_executor_node]: [ATTEMPT 3/5] FAILED (PLACE_ROLLOUT) — resetting scene and trying next candidate
[grasp_executor-1] [INFO] [1781387484.636279341] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781387484.665574908] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387484.692629746] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387485.233769765] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781387485.243665387] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387485.244647808] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387485.565108962] [grasp_executor_node]: [RESET] Arm at home.
[grasp_executor-1] [INFO] [1781387485.632405006] [grasp_executor_node]: [RESET] Object "target_obj" reset to [0.579, -0.244, -0.003]
[grasp_executor-1] [INFO] [1781387485.665382936] [grasp_executor_node]: ============================================================
[grasp_executor-1] [ATTEMPT 4/5] score=0.77 inst=0 cls=2(can) pos=[0.562,-0.231,0.138]
[grasp_executor-1] [INFO] [1781387485.673829520] [grasp_executor_node]: [PLAN] score=0.771  inst=0  cls=2(can)  width=0.0800m
[grasp_executor-1]        icgnet_tcp  = [0.5622, -0.2313, 0.1383]
[grasp_executor-1]        contact_pos = [0.5778, -0.2444, 0.0982]  (forward_offset=4.5cm)
[grasp_executor-1]        pre_pos     = [0.5205, -0.1963, 0.2453]
[grasp_executor-1]        lift_pos    = [0.5778, -0.2444, 0.2782]
[grasp_executor-1]        approach    = [0.3473, -0.2920, -0.8911]  (27.0° from vertical)
[grasp_executor-1] [INFO] [1781387485.680373144] [grasp_executor_node]: [GRIPPER] pre-grasp opening=80.0mm (ICGNet=80.0mm, per finger=40.0mm) [CAPPED at max_finger_pos=40mm/side]
[grasp_executor-1] [WARN] [1781387485.685516075] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387485.686400751] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387486.320700624] [grasp_executor_node]: [STEP 1/5] PRE-GRASP → [0.521, -0.196, 0.245]
[grasp_executor-1] [WARN] [1781387486.322277132] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387486.323146258] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387493.675256748] [grasp_executor_node]: [STEP 1/5] Pre-grasp reached in 7.35s
[grasp_executor-1] [WARN] [1781387494.522994877] [grasp_executor_node]: [CO] 'icgnet_inst_0' not found in planning scene
[grasp_executor-1] [INFO] [1781387494.524236537] [grasp_executor_node]: [CO] Removed 'icgnet_inst_0' from scene for approach
[grasp_executor-1] [INFO] [1781387494.526366216] [grasp_executor_node]: [STEP 2/5] CARTESIAN APPROACH → [0.578, -0.244, 0.098]  (vel=0.08)
[grasp_executor-1] [WARN] [1781387494.553634633] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387494.554560475] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387514.411682240] [grasp_executor_node]: [STEP 2/5] Contact surface reached in 19.87s
[grasp_executor-1] [INFO] [1781387514.412834782] [grasp_executor_node]: [STEP 3/5] CLOSING GRIPPER
[grasp_executor-1] [WARN] [1781387514.413930932] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387514.414962104] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [WARN] [1781387515.568508049] [grasp_executor_node]: [STEP 3/5] Gripper barely closed (gap=40.1mm > 40mm) — object tipped or controller aborted
[grasp_executor-1] [WARN] [1781387515.569556499] [grasp_executor_node]: [ATTEMPT 4/5] FAILED (OBJECT_TIPPED) — resetting scene and trying next candidate
[grasp_executor-1] [WARN] [1781387515.620448245] [grasp_executor_node]: [CO] 'icgnet_inst_0' not found in planning scene
[grasp_executor-1] [INFO] [1781387515.791641008] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781387515.813131211] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387515.815282383] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387516.354993694] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781387516.376346521] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387516.377233320] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387525.271928332] [grasp_executor_node]: [RESET] Arm at home.
[grasp_executor-1] [INFO] [1781387525.301036271] [grasp_executor_node]: [RESET] Object "target_obj" reset to [0.579, -0.244, -0.003]
[grasp_executor-1] [INFO] [1781387525.301941296] [grasp_executor_node]: ============================================================
[grasp_executor-1] [ATTEMPT 5/5] score=0.77 inst=0 cls=2(can) pos=[0.564,-0.231,0.139]
[grasp_executor-1] [INFO] [1781387525.323016213] [grasp_executor_node]: [PLAN] score=0.770  inst=0  cls=2(can)  width=0.0800m
[grasp_executor-1]        icgnet_tcp  = [0.5636, -0.2311, 0.1386]
[grasp_executor-1]        contact_pos = [0.5779, -0.2445, 0.0981]  (forward_offset=4.5cm)
[grasp_executor-1]        pre_pos     = [0.5255, -0.1955, 0.2467]
[grasp_executor-1]        lift_pos    = [0.5779, -0.2445, 0.2781]
[grasp_executor-1]        approach    = [0.3172, -0.2974, -0.9005]  (25.8° from vertical)
[grasp_executor-1] [INFO] [1781387525.343984987] [grasp_executor_node]: [GRIPPER] pre-grasp opening=80.0mm (ICGNet=80.0mm, per finger=40.0mm) [CAPPED at max_finger_pos=40mm/side]
[grasp_executor-1] [WARN] [1781387525.358522329] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387525.395911488] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387526.036794341] [grasp_executor_node]: [STEP 1/5] PRE-GRASP → [0.526, -0.195, 0.247]
[grasp_executor-1] [WARN] [1781387526.064628043] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387526.065577585] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387534.974277117] [grasp_executor_node]: [STEP 1/5] Pre-grasp reached in 8.92s
[grasp_executor-1] [WARN] [1781387534.018349115] [grasp_executor_node]: [CO] 'icgnet_inst_0' not found in planning scene
[grasp_executor-1] [INFO] [1781387534.037516452] [grasp_executor_node]: [CO] Removed 'icgnet_inst_0' from scene for approach
[grasp_executor-1] [INFO] [1781387534.038763995] [grasp_executor_node]: [STEP 2/5] CARTESIAN APPROACH → [0.578, -0.245, 0.098]  (vel=0.08)
[grasp_executor-1] [WARN] [1781387534.052964004] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387534.063126023] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387541.281539045] [grasp_executor_node]: [STEP 2/5] Contact surface reached in 7.24s
[grasp_executor-1] [INFO] [1781387541.293823118] [grasp_executor_node]: [STEP 3/5] CLOSING GRIPPER
[grasp_executor-1] [WARN] [1781387541.295507024] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387541.312094284] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [WARN] [1781387542.251760339] [grasp_executor_node]: [STEP 3/5] Gripper barely closed (gap=40.1mm > 40mm) — object tipped or controller aborted
[grasp_executor-1] [WARN] [1781387542.252574778] [grasp_executor_node]: [ATTEMPT 5/5] FAILED (OBJECT_TIPPED) — no more candidates
[grasp_executor-1] [INFO] [1781387557.664086047] [grasp_executor_node]: ExecuteGrasp: target='bottle' max_attempts=5 skip_place=False
[grasp_executor-1] [INFO] [1781387557.697789621] [grasp_executor_node]: [RESET] Moving arm to home before inference (clean scene for ICGNet)...
[grasp_executor-1] [WARN] [1781387557.728583966] [grasp_executor_node]: [CO] 'icgnet_inst_0' not found in planning scene
[grasp_executor-1] [INFO] [1781387557.882474504] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781387557.884429363] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387557.886061706] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387562.736138041] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781387562.736766508] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387562.737752741] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387570.130409626] [grasp_executor_node]: [RESET] Arm at home.
[grasp_executor-1] [INFO] [1781387581.439209088] [grasp_executor_node]: [FILTER] total=6684 → kept=1448 (scores=[0.00–0.86]) | rejected: width=0 workspace=0 target=4974 low_prepos=262
[grasp_executor-1] [INFO] [1781387581.440530896] [grasp_executor_node]: [SELECT] 5 candidates:
[grasp_executor-1] [INFO] [1781387581.441759506] [grasp_executor_node]:   [1] score=0.86 inst=2 cls=3(bottle) pos=[0.677,0.170,0.223] approach=[0.210,-0.621,-0.756] angle_from_vertical=40.9° width=0.080
[grasp_executor-1] [INFO] [1781387581.445864993] [grasp_executor_node]:   [2] score=0.86 inst=2 cls=3(bottle) pos=[0.681,0.160,0.229] approach=[0.123,-0.397,-0.909] angle_from_vertical=24.6° width=0.080
[grasp_executor-1] [INFO] [1781387581.490238713] [grasp_executor_node]:   [3] score=0.86 inst=2 cls=3(bottle) pos=[0.674,0.178,0.213] approach=[0.279,-0.794,-0.540] angle_from_vertical=57.3° width=0.080
[grasp_executor-1] [INFO] [1781387581.518840442] [grasp_executor_node]:   [4] score=0.86 inst=2 cls=3(bottle) pos=[0.670,0.147,0.221] approach=[0.431,-0.186,-0.883] angle_from_vertical=28.0° width=0.080
[grasp_executor-1] [INFO] [1781387581.519914497] [grasp_executor_node]:   [5] score=0.86 inst=2 cls=3(bottle) pos=[0.670,0.146,0.221] approach=[0.426,-0.192,-0.884] angle_from_vertical=27.9° width=0.080
[grasp_executor-1] [INFO] [1781387581.520818553] [grasp_executor_node]: [RESET] Teleport-back target = [0.701, 0.140, -0.001] (from /model_poses)
[grasp_executor-1] [INFO] [1781387581.552442721] [grasp_executor_node]: ============================================================
[grasp_executor-1] [ATTEMPT 1/5] score=0.86 inst=2 cls=3(bottle) pos=[0.677,0.170,0.223]
[grasp_executor-1] [INFO] [1781387581.584528015] [grasp_executor_node]: [PLAN] score=0.862  inst=2  cls=3(bottle)  width=0.0800m
[grasp_executor-1]        icgnet_tcp  = [0.6769, 0.1700, 0.2226]
[grasp_executor-1]        contact_pos = [0.6864, 0.1421, 0.1886]  (forward_offset=4.5cm)
[grasp_executor-1]        pre_pos     = [0.6518, 0.2445, 0.3132]
[grasp_executor-1]        lift_pos    = [0.6864, 0.1421, 0.3686]
[grasp_executor-1]        approach    = [0.2098, -0.6206, -0.7555]  (40.9° from vertical)
[grasp_executor-1] [INFO] [1781387581.585717167] [grasp_executor_node]: [GRIPPER] pre-grasp opening=80.0mm (ICGNet=80.0mm, per finger=40.0mm) [CAPPED at max_finger_pos=40mm/side]
[grasp_executor-1] [WARN] [1781387581.587044581] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387581.588034113] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387582.227542269] [grasp_executor_node]: [STEP 1/5] PRE-GRASP → [0.652, 0.245, 0.313]
[grasp_executor-1] [WARN] [1781387582.229452993] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387582.239640067] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387590.942031082] [grasp_executor_node]: [STEP 1/5] Pre-grasp reached in 8.68s
[grasp_executor-1] [INFO] [1781387591.820152623] [grasp_executor_node]: [CO] Removed 'icgnet_inst_2' from scene for approach
[grasp_executor-1] [INFO] [1781387591.821386310] [grasp_executor_node]: [STEP 2/5] CARTESIAN APPROACH → [0.686, 0.142, 0.189]  (vel=0.08)
[grasp_executor-1] [WARN] [1781387591.856522517] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387591.882375024] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387593.685376559] [grasp_executor_node]: [STEP 2/5] Contact surface reached in 1.82s
[grasp_executor-1] [INFO] [1781387593.686391383] [grasp_executor_node]: [STEP 3/5] CLOSING GRIPPER
[grasp_executor-1] [WARN] [1781387593.702135673] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387593.703180720] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387594.596073620] [grasp_executor_node]: [STEP 3/5] Object confirmed between fingers (gap=21.4mm, range [5–40mm])
[grasp_executor-1] [INFO] [1781387594.767624293] [grasp_executor_node]: [STEP 3b] Attached 'icgnet_inst_2' to panda_hand_tcp
[grasp_executor-1] [INFO] [1781387594.818196053] [grasp_executor_node]: [STEP 4/5] CARTESIAN LIFT → [0.686, 0.142, 0.369]
[grasp_executor-1] [WARN] [1781387594.831691945] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387594.881406024] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387596.046945316] [grasp_executor_node]: [STEP 4/5] Object lifted in 1.19s
[grasp_executor-1] [INFO] [1781387596.065827919] [grasp_executor_node]: [STEP 5/7] TRANSFER → ['0.450', '-0.500', '0.350']  (joint-space, orient=target, vel=0.25)
[grasp_executor-1] [WARN] [1781387596.067416889] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387596.068292852] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387605.393440518] [grasp_executor_node]: [STEP 5/7] Transfer done in 9.30s
[grasp_executor-1] [INFO] [1781387605.433366164] [grasp_executor_node]: [STEP 6/7] LOWER → ['0.450', '-0.500', '0.260']
[grasp_executor-1] [WARN] [1781387605.456764612] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387605.462463087] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387606.719429964] [grasp_executor_node]: [STEP 6/7] Release position reached in 1.25s
[grasp_executor-1] [INFO] [1781387606.748710507] [grasp_executor_node]: [STEP 7/7] RELEASING OBJECT
[grasp_executor-1] [WARN] [1781387606.765872433] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387606.798421290] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387607.661769932] [grasp_executor_node]: [STEP 7/7] Finger gap after open: 29.7mm (~0=gripper stuck/not open, ~40=fully open)
[grasp_executor-1] [INFO] [1781387607.674110186] [grasp_executor_node]: [STEP 7/7] Detached 'icgnet_inst_2'
[grasp_executor-1] [INFO] [1781387607.674815757] [grasp_executor_node]: [RETRACT] Cartesian retract → z=0.360
[grasp_executor-1] [WARN] [1781387607.676265976] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387607.677795827] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387609.125300677] [grasp_executor_node]: [HOME] Returning to home (object released)
[grasp_executor-1] [WARN] [1781387609.143933727] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387609.154218348] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387617.964429192] [grasp_executor_node]: [HOME] Reached in 8.82s
[grasp_executor-1] [INFO] [1781387617.977571253] [grasp_executor_node]: [PLACE] SUCCESS: object in bin (pose=[0.517, -0.512, 0.026], bin=[0.450, -0.500] ±0.150)
[grasp_executor-1] [INFO] [1781387618.013596937] [grasp_executor_node]: [SUCCESS] Grasp completed on attempt 1/5
[grasp_executor-1] [INFO] [1781387629.551674333] [grasp_executor_node]: ExecuteGrasp: target='bottle' max_attempts=5 skip_place=False
[grasp_executor-1] [INFO] [1781387629.555828917] [grasp_executor_node]: [RESET] Moving arm to home before inference (clean scene for ICGNet)...
[grasp_executor-1] [INFO] [1781387629.592450445] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781387629.607151773] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387629.608257400] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387630.070910522] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781387630.072643449] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387630.074167346] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387630.310754962] [grasp_executor_node]: [RESET] Arm at home.
[grasp_executor-1] [INFO] [1781387636.784840797] [grasp_executor_node]: [FILTER] total=3414 → kept=0 (scores=—) | rejected: width=0 workspace=0 target=3414 low_prepos=0
[grasp_executor-1] [INFO] [1781387649.249072119] [grasp_executor_node]: ExecuteGrasp: target='bottle' max_attempts=5 skip_place=False
[grasp_executor-1] [INFO] [1781387649.281790965] [grasp_executor_node]: [RESET] Moving arm to home before inference (clean scene for ICGNet)...
[grasp_executor-1] [INFO] [1781387649.287270966] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781387649.288381399] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387649.290007686] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387649.720312386] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781387649.729606949] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387649.748220835] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387649.984367403] [grasp_executor_node]: [RESET] Arm at home.
[grasp_executor-1] [INFO] [1781387657.263028891] [grasp_executor_node]: [FILTER] total=5502 → kept=0 (scores=—) | rejected: width=0 workspace=4 target=5498 low_prepos=0
[grasp_executor-1] [INFO] [1781387670.709232963] [grasp_executor_node]: ExecuteGrasp: target='cylindric' max_attempts=5 skip_place=False
[grasp_executor-1] [INFO] [1781387670.735046232] [grasp_executor_node]: [RESET] Moving arm to home before inference (clean scene for ICGNet)...
[grasp_executor-1] [INFO] [1781387670.738285448] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781387670.739618112] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387670.741715182] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387671.177123763] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781387671.189352436] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387671.190682977] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387671.607750512] [grasp_executor_node]: [RESET] Arm at home.
[grasp_executor-1] [INFO] [1781387676.696025926] [grasp_executor_node]: [FILTER] total=2748 → kept=0 (scores=—) | rejected: width=0 workspace=0 target=2748 low_prepos=0
[grasp_executor-1] [INFO] [1781387690.053014837] [grasp_executor_node]: ExecuteGrasp: target='cylindric' max_attempts=5 skip_place=False
[grasp_executor-1] [INFO] [1781387690.058777884] [grasp_executor_node]: [RESET] Moving arm to home before inference (clean scene for ICGNet)...
[grasp_executor-1] [INFO] [1781387690.060055896] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781387690.061256878] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387690.104467628] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387690.653568431] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781387690.654788538] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387690.655908371] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387691.122600492] [grasp_executor_node]: [RESET] Arm at home.
[grasp_executor-1] [INFO] [1781387703.674359982] [grasp_executor_node]: [FILTER] total=8028 → kept=875 (scores=[0.00–0.58]) | rejected: width=0 workspace=0 target=6300 low_prepos=853
[grasp_executor-1] [INFO] [1781387703.694184638] [grasp_executor_node]: [SELECT] 5 candidates:
[grasp_executor-1] [INFO] [1781387703.719686029] [grasp_executor_node]:   [1] score=0.58 inst=2 cls=4(cylindric) pos=[0.692,-0.013,0.203] approach=[-0.585,-0.811,0.000] angle_from_vertical=90.0° width=0.080
[grasp_executor-1] [INFO] [1781387703.721098783] [grasp_executor_node]:   [2] score=0.57 inst=2 cls=4(cylindric) pos=[0.696,-0.017,0.200] approach=[-0.681,-0.732,0.000] angle_from_vertical=90.0° width=0.080
[grasp_executor-1] [INFO] [1781387703.759703845] [grasp_executor_node]:   [3] score=0.57 inst=2 cls=4(cylindric) pos=[0.697,-0.019,0.200] approach=[-0.706,-0.708,0.000] angle_from_vertical=90.0° width=0.080
[grasp_executor-1] [INFO] [1781387703.778199309] [grasp_executor_node]:   [4] score=0.55 inst=2 cls=4(cylindric) pos=[0.695,-0.020,0.213] approach=[-0.677,-0.680,-0.282] angle_from_vertical=73.6° width=0.080
[grasp_executor-1] [INFO] [1781387703.779731802] [grasp_executor_node]:   [5] score=0.55 inst=2 cls=4(cylindric) pos=[0.691,-0.015,0.216] approach=[-0.560,-0.779,-0.282] angle_from_vertical=73.6° width=0.080
[grasp_executor-1] [INFO] [1781387703.780719812] [grasp_executor_node]: [RESET] Teleport-back target = [0.662, -0.048, 0.110] (from /model_poses)
[grasp_executor-1] [INFO] [1781387703.788443002] [grasp_executor_node]: ============================================================
[grasp_executor-1] [ATTEMPT 1/5] score=0.58 inst=2 cls=4(cylindric) pos=[0.692,-0.013,0.203]
[grasp_executor-1] [INFO] [1781387703.819361479] [grasp_executor_node]: [PLAN] score=0.581  inst=2  cls=4(cylindric)  width=0.0800m
[grasp_executor-1]        icgnet_tcp  = [0.6918, -0.0134, 0.2031]
[grasp_executor-1]        contact_pos = [0.6655, -0.0499, 0.2031]  (forward_offset=4.5cm)
[grasp_executor-1]        pre_pos     = [0.7620, 0.0839, 0.2031]
[grasp_executor-1]        lift_pos    = [0.6655, -0.0499, 0.3831]
[grasp_executor-1]        approach    = [-0.5849, -0.8111, 0.0000]  (90.0° from vertical)
[grasp_executor-1] [INFO] [1781387703.836960925] [grasp_executor_node]: [GRIPPER] pre-grasp opening=80.0mm (ICGNet=80.0mm, per finger=40.0mm) [CAPPED at max_finger_pos=40mm/side]
[grasp_executor-1] [WARN] [1781387703.882692081] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387703.916895066] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387704.438423953] [grasp_executor_node]: [STEP 1/5] PRE-GRASP → [0.762, 0.084, 0.203]
[grasp_executor-1] [WARN] [1781387704.439712592] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387704.440621737] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [WARN] [1781387709.631868072] [grasp_executor_node]: Planning failed! Error code: FAILURE
[grasp_executor-1] [WARN] [1781387709.670294699] [grasp_executor_node]: Cannot execute motion because the provided/planned trajectory is invalid.
[grasp_executor-1] [WARN] [1781387709.671265207] [grasp_executor_node]: Cannot wait until motion is executed (no motion is in progress).
[grasp_executor-1] [WARN] [1781387709.672964609] [grasp_executor_node]: [STEP 1/5] PRE-GRASP FAILED in 5.23s — aborting this candidate
[grasp_executor-1] [WARN] [1781387709.674889937] [grasp_executor_node]: [ATTEMPT 1/5] FAILED (PREGRASP_PLAN_FAIL) — resetting scene and trying next candidate
[grasp_executor-1] [INFO] [1781387709.879512677] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781387709.880712205] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387709.881729670] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387710.205243040] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781387710.219595436] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387710.244553625] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387710.684265809] [grasp_executor_node]: [RESET] Arm at home.
[grasp_executor-1] [INFO] [1781387710.685664228] [grasp_executor_node]: [RESET] Object undisturbed (<3cm from target) — skipping teleport.
[grasp_executor-1] [INFO] [1781387710.858050675] [grasp_executor_node]: [RESET] Re-added CO 'icgnet_inst_2' to scene.
[grasp_executor-1] [INFO] [1781387710.859480691] [grasp_executor_node]: ============================================================
[grasp_executor-1] [ATTEMPT 2/5] score=0.57 inst=2 cls=4(cylindric) pos=[0.696,-0.017,0.200]
[grasp_executor-1] [INFO] [1781387710.893717799] [grasp_executor_node]: [PLAN] score=0.573  inst=2  cls=4(cylindric)  width=0.0800m
[grasp_executor-1]        icgnet_tcp  = [0.6957, -0.0173, 0.1997]
[grasp_executor-1]        contact_pos = [0.6651, -0.0503, 0.1997]  (forward_offset=4.5cm)
[grasp_executor-1]        pre_pos     = [0.7774, 0.0706, 0.1997]
[grasp_executor-1]        lift_pos    = [0.6651, -0.0503, 0.3797]
[grasp_executor-1]        approach    = [-0.6808, -0.7325, 0.0000]  (90.0° from vertical)
[grasp_executor-1] [INFO] [1781387710.900983594] [grasp_executor_node]: [GRIPPER] pre-grasp opening=80.0mm (ICGNet=80.0mm, per finger=40.0mm) [CAPPED at max_finger_pos=40mm/side]
[grasp_executor-1] [WARN] [1781387710.948262886] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387710.957582084] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387711.474694718] [grasp_executor_node]: [STEP 1/5] PRE-GRASP → [0.777, 0.071, 0.200]
[grasp_executor-1] [WARN] [1781387711.496237423] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387711.520066970] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [WARN] [1781387716.746447912] [grasp_executor_node]: Planning failed! Error code: FAILURE
[grasp_executor-1] [WARN] [1781387716.769209809] [grasp_executor_node]: Cannot execute motion because the provided/planned trajectory is invalid.
[grasp_executor-1] [WARN] [1781387716.796766781] [grasp_executor_node]: Cannot wait until motion is executed (no motion is in progress).
[grasp_executor-1] [WARN] [1781387716.798059734] [grasp_executor_node]: [STEP 1/5] PRE-GRASP FAILED in 5.32s — aborting this candidate
[grasp_executor-1] [WARN] [1781387716.806857840] [grasp_executor_node]: [ATTEMPT 2/5] FAILED (PREGRASP_PLAN_FAIL) — resetting scene and trying next candidate
[grasp_executor-1] [INFO] [1781387717.029933553] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781387717.031416936] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387717.035306575] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387717.471353625] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781387717.472729746] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387717.473881410] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387717.697955344] [grasp_executor_node]: [RESET] Arm at home.
[grasp_executor-1] [INFO] [1781387717.747305475] [grasp_executor_node]: [RESET] Object undisturbed (<3cm from target) — skipping teleport.
[grasp_executor-1] [INFO] [1781387717.924893325] [grasp_executor_node]: [RESET] Re-added CO 'icgnet_inst_2' to scene.
[grasp_executor-1] [INFO] [1781387717.926018992] [grasp_executor_node]: ============================================================
[grasp_executor-1] [ATTEMPT 3/5] score=0.57 inst=2 cls=4(cylindric) pos=[0.697,-0.019,0.200]
[grasp_executor-1] [INFO] [1781387717.929753604] [grasp_executor_node]: [PLAN] score=0.567  inst=2  cls=4(cylindric)  width=0.0800m
[grasp_executor-1]        icgnet_tcp  = [0.6966, -0.0188, 0.2000]
[grasp_executor-1]        contact_pos = [0.6648, -0.0507, 0.2000]  (forward_offset=4.5cm)
[grasp_executor-1]        pre_pos     = [0.7813, 0.0661, 0.2000]
[grasp_executor-1]        lift_pos    = [0.6648, -0.0507, 0.3800]
[grasp_executor-1]        approach    = [-0.7060, -0.7082, 0.0000]  (90.0° from vertical)
[grasp_executor-1] [INFO] [1781387717.951763082] [grasp_executor_node]: [GRIPPER] pre-grasp opening=80.0mm (ICGNet=80.0mm, per finger=40.0mm) [CAPPED at max_finger_pos=40mm/side]
[grasp_executor-1] [WARN] [1781387717.978647951] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387717.980091388] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387716.662523371] [grasp_executor_node]: [STEP 1/5] PRE-GRASP → [0.781, 0.066, 0.200]
[grasp_executor-1] [WARN] [1781387716.677182091] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387716.678349712] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [WARN] [1781387721.915822234] [grasp_executor_node]: Planning failed! Error code: FAILURE
[grasp_executor-1] [WARN] [1781387721.916779720] [grasp_executor_node]: Cannot execute motion because the provided/planned trajectory is invalid.
[grasp_executor-1] [WARN] [1781387721.917973664] [grasp_executor_node]: Cannot wait until motion is executed (no motion is in progress).
[grasp_executor-1] [WARN] [1781387721.918857892] [grasp_executor_node]: [STEP 1/5] PRE-GRASP FAILED in 5.26s — aborting this candidate
[grasp_executor-1] [WARN] [1781387721.919814118] [grasp_executor_node]: [ATTEMPT 3/5] FAILED (PREGRASP_PLAN_FAIL) — resetting scene and trying next candidate
[grasp_executor-1] [INFO] [1781387722.153409307] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781387722.161276683] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387722.200660687] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387722.534830581] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781387722.536626641] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387722.547136311] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387722.873267862] [grasp_executor_node]: [RESET] Arm at home.
[grasp_executor-1] [INFO] [1781387722.875573572] [grasp_executor_node]: [RESET] Object undisturbed (<3cm from target) — skipping teleport.
[grasp_executor-1] [INFO] [1781387723.075206835] [grasp_executor_node]: [RESET] Re-added CO 'icgnet_inst_2' to scene.
[grasp_executor-1] [INFO] [1781387723.106217695] [grasp_executor_node]: ============================================================
[grasp_executor-1] [ATTEMPT 4/5] score=0.55 inst=2 cls=4(cylindric) pos=[0.695,-0.020,0.213]
[grasp_executor-1] [INFO] [1781387723.110272131] [grasp_executor_node]: [PLAN] score=0.550  inst=2  cls=4(cylindric)  width=0.0800m
[grasp_executor-1]        icgnet_tcp  = [0.6953, -0.0201, 0.2127]
[grasp_executor-1]        contact_pos = [0.6648, -0.0507, 0.2000]  (forward_offset=4.5cm)
[grasp_executor-1]        pre_pos     = [0.7766, 0.0614, 0.2465]
[grasp_executor-1]        lift_pos    = [0.6648, -0.0507, 0.3800]
[grasp_executor-1]        approach    = [-0.6773, -0.6796, -0.2817]  (73.6° from vertical)
[grasp_executor-1] [INFO] [1781387723.112170205] [grasp_executor_node]: [GRIPPER] pre-grasp opening=80.0mm (ICGNet=80.0mm, per finger=40.0mm) [CAPPED at max_finger_pos=40mm/side]
[grasp_executor-1] [WARN] [1781387723.138242714] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387723.138935509] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387723.570669745] [grasp_executor_node]: [STEP 1/5] PRE-GRASP → [0.777, 0.061, 0.246]
[grasp_executor-1] [WARN] [1781387723.597454570] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387723.599018159] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [WARN] [1781387728.755439704] [grasp_executor_node]: Planning failed! Error code: FAILURE
[grasp_executor-1] [WARN] [1781387728.756572783] [grasp_executor_node]: Cannot execute motion because the provided/planned trajectory is invalid.
[grasp_executor-1] [WARN] [1781387728.772878896] [grasp_executor_node]: Cannot wait until motion is executed (no motion is in progress).
[grasp_executor-1] [WARN] [1781387728.835485725] [grasp_executor_node]: [STEP 1/5] PRE-GRASP FAILED in 5.20s — aborting this candidate
[grasp_executor-1] [WARN] [1781387728.876170482] [grasp_executor_node]: [ATTEMPT 4/5] FAILED (PREGRASP_PLAN_FAIL) — resetting scene and trying next candidate
[grasp_executor-1] [INFO] [1781387729.125824818] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781387729.150472996] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387729.221480395] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387729.668637075] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781387729.712269544] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387729.713421837] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387730.073268567] [grasp_executor_node]: [RESET] Arm at home.
[grasp_executor-1] [INFO] [1781387730.094568239] [grasp_executor_node]: [RESET] Object undisturbed (<3cm from target) — skipping teleport.
[grasp_executor-1] [INFO] [1781387730.262181871] [grasp_executor_node]: [RESET] Re-added CO 'icgnet_inst_2' to scene.
[grasp_executor-1] [INFO] [1781387730.263358668] [grasp_executor_node]: ============================================================
[grasp_executor-1] [ATTEMPT 5/5] score=0.55 inst=2 cls=4(cylindric) pos=[0.691,-0.015,0.216]
[grasp_executor-1] [INFO] [1781387730.269729474] [grasp_executor_node]: [PLAN] score=0.548  inst=2  cls=4(cylindric)  width=0.0800m
[grasp_executor-1]        icgnet_tcp  = [0.6907, -0.0148, 0.2157]
[grasp_executor-1]        contact_pos = [0.6655, -0.0499, 0.2031]  (forward_offset=4.5cm)
[grasp_executor-1]        pre_pos     = [0.7579, 0.0787, 0.2495]
[grasp_executor-1]        lift_pos    = [0.6655, -0.0499, 0.3831]
[grasp_executor-1]        approach    = [-0.5600, -0.7791, -0.2817]  (73.6° from vertical)
[grasp_executor-1] [INFO] [1781387730.318047325] [grasp_executor_node]: [GRIPPER] pre-grasp opening=80.0mm (ICGNet=80.0mm, per finger=40.0mm) [CAPPED at max_finger_pos=40mm/side]
[grasp_executor-1] [WARN] [1781387730.338398501] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387730.339647943] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387730.995750454] [grasp_executor_node]: [STEP 1/5] PRE-GRASP → [0.758, 0.079, 0.250]
[grasp_executor-1] [WARN] [1781387730.998889766] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387731.000327353] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [WARN] [1781387736.204821103] [grasp_executor_node]: Planning failed! Error code: FAILURE
[grasp_executor-1] [WARN] [1781387736.205810245] [grasp_executor_node]: Cannot execute motion because the provided/planned trajectory is invalid.
[grasp_executor-1] [WARN] [1781387736.206665409] [grasp_executor_node]: Cannot wait until motion is executed (no motion is in progress).
[grasp_executor-1] [WARN] [1781387736.207682004] [grasp_executor_node]: [STEP 1/5] PRE-GRASP FAILED in 5.21s — aborting this candidate
[grasp_executor-1] [WARN] [1781387736.225564887] [grasp_executor_node]: [ATTEMPT 5/5] FAILED (PREGRASP_PLAN_FAIL) — no more candidates
[grasp_executor-1] [INFO] [1781387747.233452186] [grasp_executor_node]: ExecuteGrasp: target='cylindric' max_attempts=5 skip_place=False
[grasp_executor-1] [INFO] [1781387747.273412662] [grasp_executor_node]: [RESET] Moving arm to home before inference (clean scene for ICGNet)...
[grasp_executor-1] [INFO] [1781387747.571197776] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781387747.572228195] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387747.573034339] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387747.888255209] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781387747.889649652] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387747.925959450] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387748.245572748] [grasp_executor_node]: [RESET] Arm at home.
[grasp_executor-1] [INFO] [1781387748.420926895] [grasp_executor_node]: [RESET] Re-added CO 'icgnet_inst_2' to scene.
[grasp_executor-1] [INFO] [1781387757.370977584] [grasp_executor_node]: [FILTER] total=5160 → kept=0 (scores=—) | rejected: width=0 workspace=0 target=5160 low_prepos=0
[grasp_executor-1] [INFO] [1781387771.459743071] [grasp_executor_node]: ExecuteGrasp: target='ball' max_attempts=5 skip_place=False
[grasp_executor-1] [INFO] [1781387771.470513397] [grasp_executor_node]: [RESET] Moving arm to home before inference (clean scene for ICGNet)...
[grasp_executor-1] [INFO] [1781387771.640464861] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781387771.641444481] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387771.642386660] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387771.975744682] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781387771.984112706] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387771.992922944] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387772.305237212] [grasp_executor_node]: [RESET] Arm at home.
[grasp_executor-1] [INFO] [1781387772.499830859] [grasp_executor_node]: [RESET] Re-added CO 'icgnet_inst_2' to scene.
[grasp_executor-1] [INFO] [1781387776.028964172] [grasp_executor_node]: [FILTER] total=744 → kept=672 (scores=[0.00–0.00]) | rejected: width=0 workspace=0 target=0 low_prepos=72
[grasp_executor-1] [INFO] [1781387776.029947223] [grasp_executor_node]: [SELECT] 5 candidates:
[grasp_executor-1] [INFO] [1781387776.056387300] [grasp_executor_node]:   [1] score=0.00 inst=0 cls=5(ball) pos=[0.495,-0.175,0.062] approach=[-0.157,0.848,-0.507] angle_from_vertical=59.5° width=0.080
[grasp_executor-1] [INFO] [1781387776.057617913] [grasp_executor_node]:   [2] score=0.00 inst=0 cls=5(ball) pos=[0.495,-0.175,0.063] approach=[-0.142,0.844,-0.518] angle_from_vertical=58.8° width=0.080
[grasp_executor-1] [INFO] [1781387776.058704961] [grasp_executor_node]:   [3] score=0.00 inst=0 cls=5(ball) pos=[0.495,-0.176,0.062] approach=[-0.148,0.858,-0.493] angle_from_vertical=60.5° width=0.080
[grasp_executor-1] [INFO] [1781387776.059664631] [grasp_executor_node]:   [4] score=0.00 inst=0 cls=5(ball) pos=[0.499,-0.173,0.062] approach=[-0.231,0.830,-0.508] angle_from_vertical=59.5° width=0.080
[grasp_executor-1] [INFO] [1781387776.060574789] [grasp_executor_node]:   [5] score=0.00 inst=0 cls=5(ball) pos=[0.492,-0.167,0.072] approach=[-0.084,0.679,-0.730] angle_from_vertical=43.2° width=0.080
[grasp_executor-1] [INFO] [1781387776.061333025] [grasp_executor_node]: [RESET] Teleport-back target = [0.485, -0.138, 0.038] (from /model_poses)
[grasp_executor-1] [INFO] [1781387776.062348238] [grasp_executor_node]: ============================================================
[grasp_executor-1] [ATTEMPT 1/5] score=0.00 inst=0 cls=5(ball) pos=[0.495,-0.175,0.062]
[grasp_executor-1] [INFO] [1781387776.076233042] [grasp_executor_node]: [PLAN] score=0.000  inst=0  cls=5(ball)  width=0.0800m
[grasp_executor-1]        icgnet_tcp  = [0.4955, -0.1747, 0.0620]
[grasp_executor-1]        contact_pos = [0.4884, -0.1365, 0.0392]  (forward_offset=4.5cm)
[grasp_executor-1]        pre_pos     = [0.5144, -0.2764, 0.1228]
[grasp_executor-1]        lift_pos    = [0.4884, -0.1365, 0.2192]
[grasp_executor-1]        approach    = [-0.1574, 0.8475, -0.5069]  (59.5° from vertical)
[grasp_executor-1] [INFO] [1781387776.077502905] [grasp_executor_node]: [GRIPPER] pre-grasp opening=80.0mm (ICGNet=80.0mm, per finger=40.0mm) [CAPPED at max_finger_pos=40mm/side]
[grasp_executor-1] [WARN] [1781387776.078184756] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387776.078867454] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387776.630815873] [grasp_executor_node]: [STEP 1/5] PRE-GRASP → [0.514, -0.276, 0.123]
[grasp_executor-1] [WARN] [1781387776.632128265] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387776.632889463] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [WARN] [1781387781.849473614] [grasp_executor_node]: Planning failed! Error code: FAILURE
[grasp_executor-1] [WARN] [1781387781.876064978] [grasp_executor_node]: Cannot execute motion because the provided/planned trajectory is invalid.
[grasp_executor-1] [WARN] [1781387781.879007573] [grasp_executor_node]: Cannot wait until motion is executed (no motion is in progress).
[grasp_executor-1] [WARN] [1781387781.880274684] [grasp_executor_node]: [STEP 1/5] PRE-GRASP FAILED in 5.25s — aborting this candidate
[grasp_executor-1] [WARN] [1781387781.881545922] [grasp_executor_node]: [ATTEMPT 1/5] FAILED (PREGRASP_PLAN_FAIL) — resetting scene and trying next candidate
[grasp_executor-1] [WARN] [1781387781.923480798] [grasp_executor_node]: [CO] 'icgnet_inst_0' not found in planning scene
[grasp_executor-1] [INFO] [1781387782.080940266] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781387782.102590244] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387782.104423784] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387782.436788599] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781387782.439433485] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387782.458680848] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387782.833377060] [grasp_executor_node]: [RESET] Arm at home.
[grasp_executor-1] [INFO] [1781387782.844888766] [grasp_executor_node]: [RESET] Object undisturbed (<3cm from target) — skipping teleport.
[grasp_executor-1] [INFO] [1781387782.855571672] [grasp_executor_node]: ============================================================
[grasp_executor-1] [ATTEMPT 2/5] score=0.00 inst=0 cls=5(ball) pos=[0.495,-0.175,0.063]
[grasp_executor-1] [INFO] [1781387782.885245485] [grasp_executor_node]: [PLAN] score=0.000  inst=0  cls=5(ball)  width=0.0800m
[grasp_executor-1]        icgnet_tcp  = [0.4946, -0.1746, 0.0628]
[grasp_executor-1]        contact_pos = [0.4882, -0.1366, 0.0395]  (forward_offset=4.5cm)
[grasp_executor-1]        pre_pos     = [0.5116, -0.2758, 0.1250]
[grasp_executor-1]        lift_pos    = [0.4882, -0.1366, 0.2195]
[grasp_executor-1]        approach    = [-0.1415, 0.8437, -0.5179]  (58.8° from vertical)
[grasp_executor-1] [INFO] [1781387782.889659317] [grasp_executor_node]: [GRIPPER] pre-grasp opening=80.0mm (ICGNet=80.0mm, per finger=40.0mm) [CAPPED at max_finger_pos=40mm/side]
[grasp_executor-1] [WARN] [1781387782.895827614] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387782.912035054] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387783.337990906] [grasp_executor_node]: [STEP 1/5] PRE-GRASP → [0.512, -0.276, 0.125]
[grasp_executor-1] [WARN] [1781387783.339185135] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387783.339799022] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387790.300220958] [grasp_executor_node]: [STEP 1/5] Pre-grasp reached in 6.96s
[grasp_executor-1] [WARN] [1781387791.145012037] [grasp_executor_node]: [CO] 'icgnet_inst_0' not found in planning scene
[grasp_executor-1] [INFO] [1781387791.147195785] [grasp_executor_node]: [CO] Removed 'icgnet_inst_0' from scene for approach
[grasp_executor-1] [INFO] [1781387791.160268385] [grasp_executor_node]: [STEP 2/5] CARTESIAN APPROACH → [0.488, -0.137, 0.040]  (vel=0.08)
[grasp_executor-1] [WARN] [1781387791.169320505] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387791.170573858] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387793.353098460] [grasp_executor_node]: [STEP 2/5] Contact surface reached in 2.17s
[grasp_executor-1] [INFO] [1781387793.354736774] [grasp_executor_node]: [STEP 3/5] CLOSING GRIPPER
[grasp_executor-1] [WARN] [1781387793.356039095] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387793.356945113] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [WARN] [1781387794.220373086] [grasp_executor_node]: [STEP 3/5] Gripper fully closed (gap=0.0mm < 5mm) — missed object
[grasp_executor-1] [WARN] [1781387794.221791150] [grasp_executor_node]: [ATTEMPT 2/5] FAILED (GRASP_MISS) — resetting scene and trying next candidate
[grasp_executor-1] [WARN] [1781387794.289235934] [grasp_executor_node]: [CO] 'icgnet_inst_0' not found in planning scene
[grasp_executor-1] [INFO] [1781387794.446899210] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781387794.447803214] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387794.448582147] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387795.267052549] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781387795.268178923] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387795.268971636] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387802.313119188] [grasp_executor_node]: [RESET] Arm at home.
[grasp_executor-1] [INFO] [1781387802.347540586] [grasp_executor_node]: [RESET] Object "target_obj" reset to [0.485, -0.138, 0.038]
[grasp_executor-1] [INFO] [1781387802.359256952] [grasp_executor_node]: ============================================================
[grasp_executor-1] [ATTEMPT 3/5] score=0.00 inst=0 cls=5(ball) pos=[0.495,-0.176,0.062]
[grasp_executor-1] [INFO] [1781387802.373557591] [grasp_executor_node]: [PLAN] score=0.000  inst=0  cls=5(ball)  width=0.0800m
[grasp_executor-1]        icgnet_tcp  = [0.4950, -0.1755, 0.0616]
[grasp_executor-1]        contact_pos = [0.4884, -0.1369, 0.0395]  (forward_offset=4.5cm)
[grasp_executor-1]        pre_pos     = [0.5128, -0.2784, 0.1207]
[grasp_executor-1]        lift_pos    = [0.4884, -0.1369, 0.2195]
[grasp_executor-1]        approach    = [-0.1480, 0.8576, -0.4926]  (60.5° from vertical)
[grasp_executor-1] [INFO] [1781387802.409246880] [grasp_executor_node]: [GRIPPER] pre-grasp opening=80.0mm (ICGNet=80.0mm, per finger=40.0mm) [CAPPED at max_finger_pos=40mm/side]
[grasp_executor-1] [WARN] [1781387802.420990245] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387802.429020782] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387802.860322372] [grasp_executor_node]: [STEP 1/5] PRE-GRASP → [0.513, -0.278, 0.121]
[grasp_executor-1] [WARN] [1781387802.885596642] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387802.927715168] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [WARN] [1781387808.081632188] [grasp_executor_node]: Planning failed! Error code: FAILURE
[grasp_executor-1] [WARN] [1781387808.082657418] [grasp_executor_node]: Cannot execute motion because the provided/planned trajectory is invalid.
[grasp_executor-1] [WARN] [1781387808.083676088] [grasp_executor_node]: Cannot wait until motion is executed (no motion is in progress).
[grasp_executor-1] [WARN] [1781387808.084999787] [grasp_executor_node]: [STEP 1/5] PRE-GRASP FAILED in 5.22s — aborting this candidate
[grasp_executor-1] [WARN] [1781387808.086298623] [grasp_executor_node]: [ATTEMPT 3/5] FAILED (PREGRASP_PLAN_FAIL) — resetting scene and trying next candidate
[grasp_executor-1] [WARN] [1781387808.155544164] [grasp_executor_node]: [CO] 'icgnet_inst_0' not found in planning scene
[grasp_executor-1] [INFO] [1781387808.309760303] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781387808.310786379] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387808.311643594] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387808.640291201] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781387808.654507614] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387808.661708557] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387808.881167714] [grasp_executor_node]: [RESET] Arm at home.
[grasp_executor-1] [INFO] [1781387808.903603423] [grasp_executor_node]: [RESET] Object "target_obj" reset to [0.485, -0.138, 0.038]
[grasp_executor-1] [INFO] [1781387808.912139598] [grasp_executor_node]: ============================================================
[grasp_executor-1] [ATTEMPT 4/5] score=0.00 inst=0 cls=5(ball) pos=[0.499,-0.173,0.062]
[grasp_executor-1] [INFO] [1781387808.932532042] [grasp_executor_node]: [PLAN] score=0.000  inst=0  cls=5(ball)  width=0.0800m
[grasp_executor-1]        icgnet_tcp  = [0.4985, -0.1735, 0.0621]
[grasp_executor-1]        contact_pos = [0.4881, -0.1361, 0.0393]  (forward_offset=4.5cm)
[grasp_executor-1]        pre_pos     = [0.5262, -0.2731, 0.1231]
[grasp_executor-1]        lift_pos    = [0.4881, -0.1361, 0.2193]
[grasp_executor-1]        approach    = [-0.2307, 0.8301, -0.5076]  (59.5° from vertical)
[grasp_executor-1] [INFO] [1781387808.943087359] [grasp_executor_node]: [GRIPPER] pre-grasp opening=80.0mm (ICGNet=80.0mm, per finger=40.0mm) [CAPPED at max_finger_pos=40mm/side]
[grasp_executor-1] [WARN] [1781387808.948191501] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387808.949455845] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387807.691396725] [grasp_executor_node]: [STEP 1/5] PRE-GRASP → [0.526, -0.273, 0.123]
[grasp_executor-1] [WARN] [1781387807.693747453] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387807.695149447] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [WARN] [1781387812.942731433] [grasp_executor_node]: Planning failed! Error code: FAILURE
[grasp_executor-1] [WARN] [1781387812.946657883] [grasp_executor_node]: Cannot execute motion because the provided/planned trajectory is invalid.
[grasp_executor-1] [WARN] [1781387812.947473835] [grasp_executor_node]: Cannot wait until motion is executed (no motion is in progress).
[grasp_executor-1] [WARN] [1781387812.954011499] [grasp_executor_node]: [STEP 1/5] PRE-GRASP FAILED in 5.26s — aborting this candidate
[grasp_executor-1] [WARN] [1781387812.954973353] [grasp_executor_node]: [ATTEMPT 4/5] FAILED (PREGRASP_PLAN_FAIL) — resetting scene and trying next candidate
[grasp_executor-1] [WARN] [1781387813.010040358] [grasp_executor_node]: [CO] 'icgnet_inst_0' not found in planning scene
[grasp_executor-1] [INFO] [1781387813.164659077] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781387813.165526766] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387813.166289076] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387813.499578148] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781387813.529104024] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387813.534313440] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387813.973253973] [grasp_executor_node]: [RESET] Arm at home.
[grasp_executor-1] [INFO] [1781387813.995633818] [grasp_executor_node]: [RESET] Object "target_obj" reset to [0.485, -0.138, 0.038]
[grasp_executor-1] [INFO] [1781387814.004274315] [grasp_executor_node]: ============================================================
[grasp_executor-1] [ATTEMPT 5/5] score=0.00 inst=0 cls=5(ball) pos=[0.492,-0.167,0.072]
[grasp_executor-1] [INFO] [1781387814.007424806] [grasp_executor_node]: [PLAN] score=0.000  inst=0  cls=5(ball)  width=0.0800m
[grasp_executor-1]        icgnet_tcp  = [0.4922, -0.1673, 0.0721]
[grasp_executor-1]        contact_pos = [0.4884, -0.1367, 0.0392]  (forward_offset=4.5cm)
[grasp_executor-1]        pre_pos     = [0.5022, -0.2487, 0.1596]
[grasp_executor-1]        lift_pos    = [0.4884, -0.1367, 0.2192]
[grasp_executor-1]        approach    = [-0.0835, 0.6788, -0.7295]  (43.2° from vertical)
[grasp_executor-1] [INFO] [1781387814.008519443] [grasp_executor_node]: [GRIPPER] pre-grasp opening=80.0mm (ICGNet=80.0mm, per finger=40.0mm) [CAPPED at max_finger_pos=40mm/side]
[grasp_executor-1] [WARN] [1781387814.009535679] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387814.011071937] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387814.542835015] [grasp_executor_node]: [STEP 1/5] PRE-GRASP → [0.502, -0.249, 0.160]
[grasp_executor-1] [WARN] [1781387814.559905567] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387814.580035408] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387821.228507392] [grasp_executor_node]: [STEP 1/5] Pre-grasp reached in 6.66s
[grasp_executor-1] [WARN] [1781387822.066960725] [grasp_executor_node]: [CO] 'icgnet_inst_0' not found in planning scene
[grasp_executor-1] [INFO] [1781387822.088579411] [grasp_executor_node]: [CO] Removed 'icgnet_inst_0' from scene for approach
[grasp_executor-1] [INFO] [1781387822.089599513] [grasp_executor_node]: [STEP 2/5] CARTESIAN APPROACH → [0.488, -0.137, 0.039]  (vel=0.08)
[grasp_executor-1] [WARN] [1781387822.115742358] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387822.137223252] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387823.971870157] [grasp_executor_node]: [STEP 2/5] Contact surface reached in 1.88s
[grasp_executor-1] [INFO] [1781387823.972670619] [grasp_executor_node]: [STEP 3/5] CLOSING GRIPPER
[grasp_executor-1] [WARN] [1781387823.973493122] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387823.974272074] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [WARN] [1781387824.796571539] [grasp_executor_node]: [STEP 3/5] Gripper fully closed (gap=0.0mm < 5mm) — missed object
[grasp_executor-1] [WARN] [1781387824.797602900] [grasp_executor_node]: [ATTEMPT 5/5] FAILED (GRASP_MISS) — no more candidates
[grasp_executor-1] [INFO] [1781387837.396249649] [grasp_executor_node]: ExecuteGrasp: target='ball' max_attempts=5 skip_place=False
[grasp_executor-1] [INFO] [1781387837.397172536] [grasp_executor_node]: [RESET] Moving arm to home before inference (clean scene for ICGNet)...
[grasp_executor-1] [WARN] [1781387837.420887249] [grasp_executor_node]: [CO] 'icgnet_inst_0' not found in planning scene
[grasp_executor-1] [INFO] [1781387837.582352214] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781387837.587256852] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387837.620488100] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387838.441167907] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781387838.442669398] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387838.445545695] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [ERROR] [1781387898.584462302] [grasp_executor_node]: wait_until_executed timed out after 60.0s — cancelling motion and resetting execution state.
[grasp_executor-1] [WARN] [1781387898.663660002] [grasp_executor_node]: Action 'execute_trajectory' was unsuccessful: STATUS_ABORTED.
[grasp_executor-1] [WARN] [1781387898.665165219] [grasp_executor_node]: [RESET] Arm home move failed (continuing anyway).
[grasp_executor-1] [INFO] [1781387900.588342262] [grasp_executor_node]: [FILTER] total=696 → kept=650 (scores=[0.00–0.00]) | rejected: width=0 workspace=0 target=0 low_prepos=46
[grasp_executor-1] [INFO] [1781387900.595242952] [grasp_executor_node]: [SELECT] 5 candidates:
[grasp_executor-1] [INFO] [1781387900.616337227] [grasp_executor_node]:   [1] score=0.00 inst=0 cls=5(ball) pos=[0.435,-0.036,0.075] approach=[0.309,-0.505,-0.806] angle_from_vertical=36.3° width=0.080
[grasp_executor-1] [INFO] [1781387900.617701600] [grasp_executor_node]:   [2] score=0.00 inst=0 cls=5(ball) pos=[0.424,-0.042,0.072] approach=[0.556,-0.378,-0.741] angle_from_vertical=42.2° width=0.080
[grasp_executor-1] [INFO] [1781387900.618824966] [grasp_executor_node]:   [3] score=0.00 inst=0 cls=5(ball) pos=[0.447,-0.032,0.075] approach=[0.038,-0.591,-0.806] angle_from_vertical=36.3° width=0.080
[grasp_executor-1] [INFO] [1781387900.619885781] [grasp_executor_node]:   [4] score=0.00 inst=0 cls=5(ball) pos=[0.461,-0.079,0.074] approach=[-0.257,0.602,-0.756] angle_from_vertical=40.9° width=0.080
[grasp_executor-1] [INFO] [1781387900.620670193] [grasp_executor_node]:   [5] score=0.00 inst=0 cls=5(ball) pos=[0.435,-0.038,0.077] approach=[0.325,-0.449,-0.832] angle_from_vertical=33.7° width=0.080
[grasp_executor-1] [INFO] [1781387900.632903383] [grasp_executor_node]: [RESET] Teleport-back target = [0.449, -0.055, 0.038] (from /model_poses)
[grasp_executor-1] [INFO] [1781387900.635523830] [grasp_executor_node]: ============================================================
[grasp_executor-1] [ATTEMPT 1/5] score=0.00 inst=0 cls=5(ball) pos=[0.435,-0.036,0.075]
[grasp_executor-1] [INFO] [1781387900.639833023] [grasp_executor_node]: [PLAN] score=0.000  inst=0  cls=5(ball)  width=0.0800m
[grasp_executor-1]        icgnet_tcp  = [0.4349, -0.0363, 0.0754]
[grasp_executor-1]        contact_pos = [0.4488, -0.0591, 0.0392]  (forward_offset=4.5cm)
[grasp_executor-1]        pre_pos     = [0.3977, 0.0242, 0.1721]
[grasp_executor-1]        lift_pos    = [0.4488, -0.0591, 0.2192]
[grasp_executor-1]        approach    = [0.3095, -0.5048, -0.8058]  (36.3° from vertical)
[grasp_executor-1] [INFO] [1781387900.653893901] [grasp_executor_node]: [GRIPPER] pre-grasp opening=80.0mm (ICGNet=80.0mm, per finger=40.0mm) [CAPPED at max_finger_pos=40mm/side]
[grasp_executor-1] [WARN] [1781387900.688766522] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387900.694714379] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387901.156882344] [grasp_executor_node]: [STEP 1/5] PRE-GRASP → [0.398, 0.024, 0.172]
[grasp_executor-1] [WARN] [1781387901.158248091] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387901.159030812] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387906.289432729] [grasp_executor_node]: [STEP 1/5] Pre-grasp reached in 5.11s
[grasp_executor-1] [INFO] [1781387907.133979709] [grasp_executor_node]: [CO] Removed 'icgnet_inst_0' from scene for approach
[grasp_executor-1] [INFO] [1781387907.135468135] [grasp_executor_node]: [STEP 2/5] CARTESIAN APPROACH → [0.449, -0.059, 0.039]  (vel=0.08)
[grasp_executor-1] [WARN] [1781387907.136882091] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387907.137846061] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [ERROR] [1781387967.386754766] [grasp_executor_node]: wait_until_executed timed out after 60.0s — cancelling motion and resetting execution state.
[grasp_executor-1] [WARN] [1781387967.453840780] [grasp_executor_node]: [STEP 2/5] APPROACH FAILED in 60.25s — aborting this candidate
[grasp_executor-1] [WARN] [1781387967.455724902] [grasp_executor_node]: [ATTEMPT 1/5] FAILED (APPROACH_FAIL) — resetting scene and trying next candidate
[grasp_executor-1] [WARN] [1781387967.460624105] [grasp_executor_node]: Action 'execute_trajectory' was unsuccessful: STATUS_ABORTED.
[grasp_executor-1] [INFO] [1781387967.607421584] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781387967.608780387] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387967.610091357] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781387967.943186612] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781387967.975970173] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781387967.988686411] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [ERROR] [1781388028.108967703] [grasp_executor_node]: wait_until_executed timed out after 60.0s — cancelling motion and resetting execution state.
[grasp_executor-1] [WARN] [1781388028.143273674] [grasp_executor_node]: [RESET] Arm home move failed (continuing anyway).
[grasp_executor-1] [INFO] [1781388028.205065403] [grasp_executor_node]: [RESET] Object "target_obj" reset to [0.449, -0.055, 0.038]
[grasp_executor-1] [WARN] [1781388028.209624646] [grasp_executor_node]: Action 'execute_trajectory' was unsuccessful: STATUS_ABORTED.
[grasp_executor-1] [INFO] [1781388028.380394362] [grasp_executor_node]: [RESET] Re-added CO 'icgnet_inst_0' to scene.
[grasp_executor-1] [INFO] [1781388028.406005930] [grasp_executor_node]: ============================================================
[grasp_executor-1] [ATTEMPT 2/5] score=0.00 inst=0 cls=5(ball) pos=[0.424,-0.042,0.072]
[grasp_executor-1] [INFO] [1781388028.410083390] [grasp_executor_node]: [PLAN] score=0.000  inst=0  cls=5(ball)  width=0.0800m
[grasp_executor-1]        icgnet_tcp  = [0.4238, -0.0421, 0.0725]
[grasp_executor-1]        contact_pos = [0.4488, -0.0591, 0.0392]  (forward_offset=4.5cm)
[grasp_executor-1]        pre_pos     = [0.3571, 0.0033, 0.1614]
[grasp_executor-1]        lift_pos    = [0.4488, -0.0591, 0.2192]
[grasp_executor-1]        approach    = [0.5557, -0.3778, -0.7406]  (42.2° from vertical)
[grasp_executor-1] [INFO] [1781388028.429835882] [grasp_executor_node]: [GRIPPER] pre-grasp opening=80.0mm (ICGNet=80.0mm, per finger=40.0mm) [CAPPED at max_finger_pos=40mm/side]
[grasp_executor-1] [WARN] [1781388028.439705947] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781388028.440756064] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781388028.768917257] [grasp_executor_node]: [STEP 1/5] PRE-GRASP → [0.357, 0.003, 0.161]
[grasp_executor-1] [WARN] [1781388028.789781688] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781388028.816975248] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [ERROR] [1781388089.171444174] [grasp_executor_node]: wait_until_executed timed out after 60.0s — cancelling motion and resetting execution state.
[grasp_executor-1] [WARN] [1781388089.234826311] [grasp_executor_node]: [STEP 1/5] PRE-GRASP FAILED in 60.40s — aborting this candidate
[grasp_executor-1] [WARN] [1781388089.241176837] [grasp_executor_node]: Action 'execute_trajectory' was unsuccessful: STATUS_ABORTED.
[grasp_executor-1] [WARN] [1781388089.241867243] [grasp_executor_node]: [ATTEMPT 2/5] FAILED (PREGRASP_PLAN_FAIL) — resetting scene and trying next candidate
[grasp_executor-1] [INFO] [1781388089.555994515] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781388089.557140501] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781388089.558195097] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781388089.771866390] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781388089.773273328] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781388089.774445759] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [ERROR] [1781388149.930369484] [grasp_executor_node]: wait_until_executed timed out after 60.0s — cancelling motion and resetting execution state.
[grasp_executor-1] [WARN] [1781388149.932959815] [grasp_executor_node]: [RESET] Arm home move failed (continuing anyway).
[grasp_executor-1] [WARN] [1781388150.004279948] [grasp_executor_node]: Action 'execute_trajectory' was unsuccessful: STATUS_ABORTED.
[grasp_executor-1] [INFO] [1781388150.006675668] [grasp_executor_node]: [RESET] Object "target_obj" reset to [0.449, -0.055, 0.038]
[grasp_executor-1] [INFO] [1781388150.184756112] [grasp_executor_node]: [RESET] Re-added CO 'icgnet_inst_0' to scene.
[grasp_executor-1] [INFO] [1781388150.210422183] [grasp_executor_node]: ============================================================
[grasp_executor-1] [ATTEMPT 3/5] score=0.00 inst=0 cls=5(ball) pos=[0.447,-0.032,0.075]
[grasp_executor-1] [INFO] [1781388150.239307579] [grasp_executor_node]: [PLAN] score=0.000  inst=0  cls=5(ball)  width=0.0800m
[grasp_executor-1]        icgnet_tcp  = [0.4471, -0.0325, 0.0754]
[grasp_executor-1]        contact_pos = [0.4488, -0.0591, 0.0392]  (forward_offset=4.5cm)
[grasp_executor-1]        pre_pos     = [0.4425, 0.0384, 0.1721]
[grasp_executor-1]        lift_pos    = [0.4488, -0.0591, 0.2192]
[grasp_executor-1]        approach    = [0.0382, -0.5909, -0.8058]  (36.3° from vertical)
[grasp_executor-1] [INFO] [1781388150.293049721] [grasp_executor_node]: [GRIPPER] pre-grasp opening=80.0mm (ICGNet=80.0mm, per finger=40.0mm) [CAPPED at max_finger_pos=40mm/side]
[grasp_executor-1] [WARN] [1781388150.327675601] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781388150.360444537] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781388150.689344920] [grasp_executor_node]: [STEP 1/5] PRE-GRASP → [0.443, 0.038, 0.172]
[grasp_executor-1] [WARN] [1781388150.691589606] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781388150.719220504] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [ERROR] [1781388210.955562249] [grasp_executor_node]: wait_until_executed timed out after 60.0s — cancelling motion and resetting execution state.
[grasp_executor-1] [WARN] [1781388211.055632037] [grasp_executor_node]: [STEP 1/5] PRE-GRASP FAILED in 60.27s — aborting this candidate
[grasp_executor-1] [WARN] [1781388211.056377247] [grasp_executor_node]: Action 'execute_trajectory' was unsuccessful: STATUS_ABORTED.
[grasp_executor-1] [WARN] [1781388211.082893067] [grasp_executor_node]: [ATTEMPT 3/5] FAILED (PREGRASP_PLAN_FAIL) — resetting scene and trying next candidate
[grasp_executor-1] [INFO] [1781388211.268765212] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781388211.269983762] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781388211.270794930] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781388211.492683841] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781388211.508907632] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781388211.521430850] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [ERROR] [1781388271.660275717] [grasp_executor_node]: wait_until_executed timed out after 60.0s — cancelling motion and resetting execution state.
[grasp_executor-1] [WARN] [1781388271.691151387] [grasp_executor_node]: [RESET] Arm home move failed (continuing anyway).
[grasp_executor-1] [WARN] [1781388271.730924673] [grasp_executor_node]: Action 'execute_trajectory' was unsuccessful: STATUS_ABORTED.
[grasp_executor-1] [INFO] [1781388271.745627650] [grasp_executor_node]: [RESET] Object "target_obj" reset to [0.449, -0.055, 0.038]
[grasp_executor-1] [INFO] [1781388271.921077572] [grasp_executor_node]: [RESET] Re-added CO 'icgnet_inst_0' to scene.
[grasp_executor-1] [INFO] [1781388271.948939879] [grasp_executor_node]: ============================================================
[grasp_executor-1] [ATTEMPT 4/5] score=0.00 inst=0 cls=5(ball) pos=[0.461,-0.079,0.074]
[grasp_executor-1] [INFO] [1781388271.960406120] [grasp_executor_node]: [PLAN] score=0.000  inst=0  cls=5(ball)  width=0.0800m
[grasp_executor-1]        icgnet_tcp  = [0.4613, -0.0791, 0.0745]
[grasp_executor-1]        contact_pos = [0.4497, -0.0520, 0.0404]  (forward_offset=4.5cm)
[grasp_executor-1]        pre_pos     = [0.4921, -0.1513, 0.1652]
[grasp_executor-1]        lift_pos    = [0.4497, -0.0520, 0.2204]
[grasp_executor-1]        approach    = [-0.2573, 0.6021, -0.7558]  (40.9° from vertical)
[grasp_executor-1] [INFO] [1781388271.961936670] [grasp_executor_node]: [GRIPPER] pre-grasp opening=80.0mm (ICGNet=80.0mm, per finger=40.0mm) [CAPPED at max_finger_pos=40mm/side]
[grasp_executor-1] [WARN] [1781388271.963373752] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781388271.985134136] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781388272.318820889] [grasp_executor_node]: [STEP 1/5] PRE-GRASP → [0.492, -0.151, 0.165]
[grasp_executor-1] [WARN] [1781388272.344153648] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781388272.347078612] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [ERROR] [1781388332.668568420] [grasp_executor_node]: wait_until_executed timed out after 60.0s — cancelling motion and resetting execution state.
[grasp_executor-1] [WARN] [1781388332.671333032] [grasp_executor_node]: [STEP 1/5] PRE-GRASP FAILED in 60.35s — aborting this candidate
[grasp_executor-1] [WARN] [1781388332.697431281] [grasp_executor_node]: [ATTEMPT 4/5] FAILED (PREGRASP_PLAN_FAIL) — resetting scene and trying next candidate
[grasp_executor-1] [WARN] [1781388332.782390776] [grasp_executor_node]: Action 'execute_trajectory' was unsuccessful: STATUS_ABORTED.
[grasp_executor-1] [INFO] [1781388332.908655838] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781388332.909386309] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781388332.909889061] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781388333.131024500] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781388333.131991243] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781388333.132853421] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [ERROR] [1781388393.332481095] [grasp_executor_node]: wait_until_executed timed out after 60.0s — cancelling motion and resetting execution state.
[grasp_executor-1] [WARN] [1781388393.356666792] [grasp_executor_node]: [RESET] Arm home move failed (continuing anyway).
[grasp_executor-1] [INFO] [1781388393.387331823] [grasp_executor_node]: [RESET] Object "target_obj" reset to [0.449, -0.055, 0.038]
[grasp_executor-1] [WARN] [1781388393.482248581] [grasp_executor_node]: Action 'execute_trajectory' was unsuccessful: STATUS_ABORTED.
[grasp_executor-1] [INFO] [1781388393.586482124] [grasp_executor_node]: [RESET] Re-added CO 'icgnet_inst_0' to scene.
[grasp_executor-1] [INFO] [1781388393.599910277] [grasp_executor_node]: ============================================================
[grasp_executor-1] [ATTEMPT 5/5] score=0.00 inst=0 cls=5(ball) pos=[0.435,-0.038,0.077]
[grasp_executor-1] [INFO] [1781388393.602568175] [grasp_executor_node]: [PLAN] score=0.000  inst=0  cls=5(ball)  width=0.0800m
[grasp_executor-1]        icgnet_tcp  = [0.4349, -0.0385, 0.0768]
[grasp_executor-1]        contact_pos = [0.4495, -0.0587, 0.0394]  (forward_offset=4.5cm)
[grasp_executor-1]        pre_pos     = [0.3959, 0.0154, 0.1767]
[grasp_executor-1]        lift_pos    = [0.4495, -0.0587, 0.2194]
[grasp_executor-1]        approach    = [0.3250, -0.4493, -0.8322]  (33.7° from vertical)
[grasp_executor-1] [INFO] [1781388393.622470261] [grasp_executor_node]: [GRIPPER] pre-grasp opening=80.0mm (ICGNet=80.0mm, per finger=40.0mm) [CAPPED at max_finger_pos=40mm/side]
[grasp_executor-1] [WARN] [1781388393.624008158] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781388393.624834944] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781388393.962822098] [grasp_executor_node]: [STEP 1/5] PRE-GRASP → [0.396, 0.015, 0.177]
[grasp_executor-1] [WARN] [1781388393.964276204] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781388393.965072270] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [ERROR] [1781388454.111449762] [grasp_executor_node]: wait_until_executed timed out after 60.0s — cancelling motion and resetting execution state.
[grasp_executor-1] [WARN] [1781388454.114124554] [grasp_executor_node]: [STEP 1/5] PRE-GRASP FAILED in 60.15s — aborting this candidate
[grasp_executor-1] [WARN] [1781388454.115442393] [grasp_executor_node]: [ATTEMPT 5/5] FAILED (PREGRASP_PLAN_FAIL) — no more candidates
[grasp_executor-1] [WARN] [1781388454.201142726] [grasp_executor_node]: Action 'execute_trajectory' was unsuccessful: STATUS_ABORTED.
[grasp_executor-1] [INFO] [1781388467.525803148] [grasp_executor_node]: ExecuteGrasp: target='ball' max_attempts=5 skip_place=False
[grasp_executor-1] [INFO] [1781388467.527489104] [grasp_executor_node]: [RESET] Moving arm to home before inference (clean scene for ICGNet)...
[grasp_executor-1] [INFO] [1781388467.728013595] [grasp_executor_node]: [RESET] Opening gripper...
[grasp_executor-1] [WARN] [1781388467.729805276] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781388467.730770855] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [INFO] [1781388468.057259139] [grasp_executor_node]: [RESET] Moving arm to home...
[grasp_executor-1] [WARN] [1781388468.058737478] [grasp_executor_node]: Joint states are not available yet!
[grasp_executor-1] [INFO] [1781388468.070494776] [grasp_executor_node]: Joint states are available now
[grasp_executor-1] [ERROR] [1781388528.194899723] [grasp_executor_node]: wait_until_executed timed out after 60.0s — cancelling motion and resetting execution state.
[grasp_executor-1] [WARN] [1781388528.242686405] [grasp_executor_node]: [RESET] Arm home move failed (continuing anyway).
[grasp_executor-1] [WARN] [1781388528.349015475] [grasp_executor_node]: Action 'execute_trajectory' was unsuccessful: STATUS_ABORTED.
[grasp_executor-1] [INFO] [1781388528.422989599] [grasp_executor_node]: [RESET] Re-added CO 'icgnet_inst_0' to scene.
[grasp_executor-1] [INFO] [1781388536.564213320] [grasp_executor_node]: [FILTER] total=822 → kept=0 (scores=—) | rejected: width=0 workspace=0 target=822 low_prepos=0