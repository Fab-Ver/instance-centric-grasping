# Testing Guide — ICGNet ROS2 Gazebo Sim Fortress

**Branch:** `main`
**Stack:** ROS2 Humble + Gazebo Sim Fortress (gz-sim 6, DART physics) + MoveIt2 + gz_ros2_control
**Last updated:** 2026-06-14

---

## Dependencies

### System (apt)

```bash
sudo apt install \
  ros-humble-pick-ik \
  ros-humble-ign-ros2-control \
  ros-humble-ros-gz \
  ros-humble-moveit \
  ros-humble-joint-trajectory-controller \
  ros-humble-joint-state-broadcaster \
  ros-humble-controller-manager
```

### Python (ML / ICGNet inference — GPU terminal only)

```bash
cd ~/instance-centric-grasping
uv sync          # creates .venv with Python 3.12 + all ML deps (torch, minkowskiengine, trimesh…)
```

---

## Build

Every new shell must source before any `ros2` command:

```bash
source /opt/ros/humble/setup.bash
source ~/instance-centric-grasping/install/setup.bash
```

Build (from workspace root):

```bash
cd ~/instance-centric-grasping
colcon build --packages-select panda_description icgnet_msgs icgnet_main pymoveit2 
source install/setup.bash
```

---

## Rendering — choose your mode before launching

The launch file exposes a `use_gpu` argument that selects the Gazebo rendering backend.
No environment variables need to be set manually — the launch file handles everything.

| `use_gpu` | Who | Backend | Notes |
|-----------|-----|---------|-------|
| `false` (default) | Everyone without a GPU | LLVMpipe (CPU software) + `OGRE_RTT_MODE=Copy` | Works on any machine; slower fps but stable |
| `true` | WSL2 + NVIDIA GPU | Mesa D3D12 hardware | Requires `d3d12_dri.so` (Mesa 23.2+) and `/usr/lib/wsl/lib/libd3d12.so` |

---

## Terminal Sequence

All terminals: start from `~/instance-centric-grasping` and source the environment first.

### T1 — Simulation + MoveIt2 + Camera bridge + TF

**Without GPU (default — works for everyone):**

```bash
cd ~/instance-centric-grasping
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch icgnet_main world.launch.py
```

> **Gazebo headless (default `headless:=true`, 2026-06-09)**: no GUI gz window. RViz is the only
> visualization — object meshes are mirrored live by `scene_visualizer` on `/icgnet/scene_meshes`.
> Pass `headless:=false` to restore the gz GUI for physics debugging.

Wait for:
- `[controller_manager]` ready
- `[move_group]` → `Ready to take commands for planning group arm.`
- `[gz_ros_bridge]` publishing (camera sensor init takes ~10 s)
- RViz opens; `[scene_visualizer]` → `SceneVisualizerNode ready`

### T2 — Spawn objects (after T1 ready, ~15 s)

Two modes — pick one:

**A — Single object (legacy, uses spawn_object):**

```bash
cd ~/instance-centric-grasping
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run icgnet_main spawn_object --ros-args -p target_class:=can
# or by exact model: -p target_type:=beer_can
```

**B — Multi-object scene (uses scene_manager):**

```bash
cd ~/instance-centric-grasping
source /opt/ros/humble/setup.bash && source install/setup.bash

# Spawns 2 cans + 2–3 random objects of OTHER classes (box, mug, ball).
# target_class / target_count only control spawn quantities — execute_grasp
# can target ANY class present in the scene, regardless of how it was spawned.
# Node stays alive to serve /icgnet/reset_scene.
ros2 run icgnet_main scene_manager --ros-args \
  -p target_class:=can -p target_count:=2

# Verify manifest (shows all objects with their semantic class):
ros2 topic echo /icgnet/scene_manifest --once

# Manual reset (teleports all objects back to spawn pose):
ros2 service call /icgnet/reset_scene std_srvs/srv/Trigger
```

Available classes: `can`, `box`, `mug`, `ball`

### T3 — Perception (choose A or B)

**A — With GPU (real ICGNet inference)**

```bash
cd ~/instance-centric-grasping
source /opt/ros/humble/setup.bash && source install/setup.bash
export PYTHONPATH=~/instance-centric-grasping/.venv/lib/python3.12/site-packages:$PYTHONPATH
ros2 launch icgnet_main icgnet_inference.launch.py
# Expected: "ICGNet loaded successfully." + "ICGNetGraspNode ready"
```

**B — Without GPU (replay saved inference data)**

> Prerequisite: generate `~/icgnet_inference_data/` first on a GPU machine (see Save Inference section).

> **Object spawn coordinates for replay**: the bundled inference data was saved with the object at
> `x=0.65, y=0.0, z=0.05`. The object **must** be spawned at exactly those coordinates in T2,
> otherwise the saved grasp poses will not match the object's actual position. Between failed
> attempts the grasp executor teleports the object back to these coordinates automatically
> (configured via `object_init_x/y/z` in `grasp_executor_params.yaml`).

```bash
cd ~/instance-centric-grasping
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch icgnet_main icgnet_replay.launch.py \
  inference_dir:=./icgnet_inference_data
# Expected: "ReplayInferenceNode ready — N grasps, M collision objects"
```

### T4 — Grasp executor

```bash
cd ~/instance-centric-grasping
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch icgnet_main grasp_execution.launch.py
# Expected: "GraspExecutorNode ready"
```

### T5 — Triggers (after T1 + T2 + T3 + T4 all up)

```bash
# Single-object (T2-A) or multi-object sweep (T2-B) — same commands:

# Step 1: run inference
ros2 service call /icgnet/compute_grasps std_srvs/srv/Trigger

# Step 2: execute — target sets WHAT to grasp, independent of how objects were spawned.
# With T2-A: grasps the single object if it matches the target class.
# With T2-B: sweeps ALL instances of the target class (including those spawned as
#            "distractors") into the bin; objects of other classes are left in place.
#            If the requested class is absent from the manifest, returns an error
#            listing which classes ARE present.
ros2 service call /icgnet/execute_grasp icgnet_msgs/srv/ExecuteGrasp "{target: 'can'}"
ros2 service call /icgnet/execute_grasp icgnet_msgs/srv/ExecuteGrasp "{target: 'box'}"
ros2 service call /icgnet/execute_grasp icgnet_msgs/srv/ExecuteGrasp "{target: 'any'}"

# Debug (lift only, no place):
ros2 service call /icgnet/execute_grasp icgnet_msgs/srv/ExecuteGrasp "{target: 'can', skip_place: true}"
```

---

## Save Inference Data (generate GPU-less replay dataset)

With T1 running and the GPU inference node (T3-A) up:

```bash
cd ~/instance-centric-grasping
source /opt/ros/humble/setup.bash && source install/setup.bash

PKG=$(ros2 pkg prefix icgnet_main)/share/icgnet_main
ros2 run icgnet_main save_inference --ros-args \
  -p object_sdf_path:=$PKG/models/can/beer_can/model.sdf \
  -p object_name:=target_obj \
  -p object_x:=0.65 -p object_y:=0.0 -p object_z:=0.05
# → saves grasps.bin, collision_N.bin, meta.json to ~/icgnet_inference_data/
# Trigger inference first: ros2 service call /icgnet/compute_grasps std_srvs/srv/Trigger
# Wait for [SAVED] in log, then Ctrl+C
```

---

## Phase 1 Evaluation — automated single-object benchmark

`scripts/run_evaluation_phase1.py` drives the full grasp pipeline automatically across many
runs and one object class at a time (no distractors — multi-object is a documented domain-gap
limitation). For each run it spawns one object, calls `/icgnet/execute_grasp`, and logs the
outcome plus the per-attempt failure reason.

### Prerequisites — terminals that must be up

Bring up **T1 + T3-A + T4** (the script handles spawn + grasp itself — you do **not** run T2 or T5):

```bash
# T1 — simulation. rviz:=false disables the RViz visualization stack (RViz + scene_visualizer)
# for a fully headless, faster benchmark (gz is already headless by default).
ros2 launch icgnet_main world.launch.py rviz:=false

# T3-A — GPU inference (REQUIRED: replay only has one saved object, no good for a multi-class run)
ros2 launch icgnet_main icgnet_inference.launch.py

# T4 — grasp executor
ros2 launch icgnet_main grasp_execution.launch.py
```

`grasp_executor` triggers `/icgnet/compute_grasps` internally on each `execute_grasp` call, so the
script never calls `compute_grasps` directly.

**Pre-check (required for ground-truth success):** `/model_poses` must be publishing —
`_object_in_bin()` reads it to decide success.

```bash
ros2 topic echo /model_poses --once   # must print a TFMessage
```

### Run

```bash
cd ~/instance-centric-grasping
source /opt/ros/humble/setup.bash && source install/setup.bash

# Default: 6 classes × 20 runs, target-driven (mug, box, can, bottle, cylindric, ball)
./scripts/run_evaluation_phase1.py

# Quick subset:
./scripts/run_evaluation_phase1.py --runs-per-class 30 --classes can ball

# Two separate experiments (single-object always; run them at different times):
./scripts/run_evaluation_phase1.py --runs-per-class 30               # mode=target (default)
./scripts/run_evaluation_phase1.py --runs-per-class 30 --mode any    # class-agnostic
```

| Option | Default | Meaning |
|--------|---------|---------|
| `--runs-per-class N` | `20` | runs per object class |
| `--classes ...` | all 6 catalog classes | which classes to evaluate |
| `--mode {target,any}` | `target` | grasp target: `target` = the spawned class (target-driven, needs correct ICGNet classification); `any` = class-agnostic (isolates segmentation+grasp from the classification step). The two modes write to separate `_<mode>_` files. |

Each class spawns exactly one model from `catalog.yaml` (deterministic, no per-run variance):
`mug`→threshold_porcelain_coffee_mug, `box`→cardboard_box, `can`→coke_can,
`bottle`→cajun_tonic_bottle, `cylindric`→pringles_can, `ball`→tennis_ball.

**Spawn workspace (2026-06-14)**: objects spawn in `x∈[0.45,0.70]`, `y∈[±0.30]`, `reach≤0.75` —
the arm's dexterous workspace AND the camera's reliable FOV (matches `scene_manager`). The previous
`x∈[0.40,0.80]`, `reach≤0.85` reached the camera frustum edge (Mask3D fails) and the Panda's
kinematic limit, inflating `PERCEPTION_NO_GRASP`/`PREGRASP_PLAN_FAIL` with confounds not attributable
to ICGNet.

**Respawn robustness (2026-06-14)**: each run does `remove → spawn` with **verification + retry**.
The harness subscribes to `/model_poses` (ground truth of which entities are in gz) and confirms the
old object is gone before spawning and the new one appears before grasping; `spawn_object` now exits
non-zero on a name collision so the harness can retry instead of grasping a **stale, never-respawned**
scene (the bug that produced bit-identical inference rows in earlier CSVs). A run that fails 3
remove+spawn cycles is logged as `SPAWN_FAIL` rather than silently corrupting the batch.

**Per-run speedup (2026-06-14)**: `spawn_object` has a `gz_server_wait` param (default `5.0` s) that
only matters on a cold start (spawn launched right after `world.launch.py`, gz-sim server still
booting). The eval loop passes `gz_server_wait:=0.0` — the server is up for the whole batch and
presence is verified via `/model_poses` — saving ~5 s/run (~15 min over a 180-run batch). Manual
T2-A spawns keep the 5 s default.

### Output — versioned under `report/`, never overwrites

All paths share the `<R>runs_<classes>_<mode>_v<N>` stem (`<N>`=0 or last+1):

- **`report/eval_<...>_v<N>.csv`** — one row per run:
  `Run_ID, Target_Class, Detected_Classes, Success, Attempts, First_Attempt, Planning_Time,
  Execution_Time, Collision_Detected, Target_Not_Found, Failure_Reason, Attempt_Reasons, Attempt_Scores`
  - `Detected_Classes` = one label per ICGNet instance **pre class-filter** (true-class→detected confusion).
  - `Planning_Time` = ICGNet inference + selection (wall-clock; dominated by GPU inference).
  - `Execution_Time` = arm/gripper motion on the **sim clock** (wall-clock is meaningless at low RTF).
  - `Failure_Reason` = run-level outcome code; `Attempt_Reasons` / `Attempt_Scores` = `;`-joined
    per-attempt codes and the ICGNet score of the proposal tried at each attempt (parallel lists).
  - `Target_Class` always records the **spawned** class (even in `mode=any`) → per-class breakdown works in both modes.
- **`report/eval_<...>_v<N>_summary.txt`** — overall + per-class GSR, avg attempts until success,
  first-attempt rate, failure-mode histograms (run-level + attempt-level), and grasp-proposal score
  by outcome (SUCCESS vs FAILED) + best-proposal score per class (score↔success correlation).
- **`report/inference_<...>_v<N>.jsonl`** — one JSONL line per run with the **full ICGNet proposal set**
  (pre-filter), grouped by instance: `instance_id, semantic_class, class_name, n_grasps, best_score,
  grasp_centroid, grasps[{score, tcp, approach_axis, inclination_deg, width}]`. Lets you quantify
  over-segmentation (instances/object) and **grasp inclination** offline without re-running the GPU.
- **`report/grasping_<...>_v<N>.jsonl`** — one JSONL line per run with per-attempt geometry + outcome:
  `attempts[{idx, instance_id, score, tcp, approach_axis, inclination_deg, width, reason}]`.

`inclination_deg` = angle between the gripper approach axis and world −Z (0° = top-down); high
inclination is the geometric driver of `PREGRASP_PLAN_FAIL`. Dumps are written by `grasp_executor`
(it has the raw pre-filter data); the harness passes `run_id` + dump paths via `ExecuteGrasp.srv`.
The three `report/` files line up by `Run_ID`.

**Failure-mode codes** (`Failure_Reason` / `Attempt_Reasons`): `SUCCESS`, `PERCEPTION_NO_GRASP`,
`PREGRASP_PLAN_FAIL`, `APPROACH_FAIL`, `GRASP_MISS`, `OBJECT_TIPPED`, `LIFT_PLAN_FAIL`, `LIFT_DROP`,
`TRANSFER_PLAN_FAIL`, `TRANSFER_DROP`, `LOWER_PLAN_FAIL`, `LOWER_DROP`, `PLACE_ROLLOUT`, `SPAWN_FAIL`.

> Notes: tall/thin objects (`cylindric`=pringles_can, `bottle`=cajun_tonic_bottle) are expected to
> show more `APPROACH_FAIL`/`GRASP_MISS` — read the per-class attempt histogram, not just GSR.
> CSV is flushed every run, so the file is usable even if the run is interrupted.

---

## Headless Verification (no GUI required)

After T1 is up, in a separate shell:

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash

# 3 controllers must be active:
ros2 control list_controllers
# joint_state_broadcaster   [joint_state_broadcaster/JointStateBroadcaster]   active
# panda_arm_controller      [joint_trajectory_controller/JointTrajectoryController] active
# panda_hand_controller     [joint_trajectory_controller/JointTrajectoryController] active

# MoveIt2 move_group:
ros2 node list | grep move_group

# FollowJointTrajectory actions:
ros2 action list | grep follow_joint_trajectory

# gz-sim scene-reset service (required for grasp_executor):
ros2 service list | grep set_pose          # /world/icgnet_world/set_pose

# Camera pointcloud (~30 Hz, appears after ~10 s):
ros2 topic hz /camera/rgbd_camera/points

# After compute_grasps trigger — ICGNet predictions:
ros2 topic echo /icgnet/grasps_rich --once

# After compute_grasps — collision objects in planning scene:
ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene \
  '{components: {components: 1}}' 2>/dev/null | grep '"id"'
```

**RViz scene_visualizer — object meshes synced to the robot (2026-06-09):**

```bash
# Marker definitions republished at ~10 Hz (motion is carried by TF, not this rate):
ros2 topic hz /icgnet/scene_meshes

# Per-object moving TF frame <entity>_viz — must appear in the tree:
ros2 run tf2_tools view_frames        # → world → target_obj_viz
ros2 topic hz /tf                      # includes object frames at ~36 Hz
```

In RViz the object mesh stays **locked to the gripper** during approach/lift/transport — no lag,
no detachment, as fluid as the robot. Each object gets its own `<entity>_viz` TF frame and a
`frame_locked` marker (same latest-TF retransform RViz uses for the RobotModel).

**Sanity check — no Classic remnants:**

```bash
grep -rIE "gazebo_ros|gazebo_msgs|spawn_entity\.py|panda_controllers" \
  src/icgnet_main src/panda_description src/icgnet_msgs src/pymoveit2
# Expected: 0 hits
```

---

## Object Management

```bash
# Delete object from gz-sim:
ros2 service call /world/icgnet_world/remove ros_gz_interfaces/srv/DeleteEntity \
  "{entity: {name: 'target_obj', type: 2}}"

# Teleport object back to init position (grasp_executor does this automatically on reset):
ros2 service call /world/icgnet_world/set_pose ros_gz_interfaces/srv/SetEntityPose \
  "{entity: {name: 'target_obj', type: 2}, pose: {position: {x: 0.65, y: 0.0, z: 0.05}}}"
```

---

## Expected Grasp Log Sequence

```
[SPAWN_POSE] Object spawn pose received: (0.65, 0.00, 0.052)
[RECON]   inst_0 → class=can (id=2)
[INSTANCES] 1 instance(s): inst_0=can(id=2, 353g) | total_grasps=353
[SCORES]  top-10: [0.807, 0.806, ...] | min=0.300 max=0.807 mean=0.559 | >0.3: 353 >0.5: 186 >0.7: 0
[FILTER]  total=353 → kept=353 (scores=[0.30–0.81]) | rejected: width=0 workspace=0 target=0
[PLAN]    score=0.807  inst=0  cls=2(can)  width=0.0660m
[GRIPPER] pre-grasp opening=73.0mm (per finger=36.5mm)
[RESET]   Re-added CO 'icgnet_inst_0' to scene.
[STEP 1/5] PRE-GRASP  → [x, y, z+0.12]   Pre-grasp reached in Xs
[CO]      Removed 'icgnet_inst_0' from scene for approach
[STEP 2/5] APPROACH   → [x, y, z]   Contact surface reached in Xs
[STEP 3/5] CLOSING GRIPPER  gap=21.3mm  ✓
[STEP 3b]  Attached 'icgnet_inst_0' to panda_hand_tcp
[STEP 4/5] LIFTING    → [x, y, z+0.18]   Object lifted in Xs
[STEP 5/7] TRANSFER   → (0.45, -0.50, 0.35)
[STEP 6/7] LOWER      → z=0.26
[STEP 7/7] OPEN GRIPPER + settle + retract + HOME
[STEP 8]   Detached and removed 'icgnet_inst_0'
[SUCCESS]  Grasp completed on attempt 1/5
```

---

## Known Issues

| Symptom | Cause | Fix / Status |
|---|---|---|
| Attempts 2–5 all `INVALID_MOTION_PLAN` after attempt 1 fails | After gap-check failure, CO was re-added while arm is at contact_pos → start state in collision | **Fixed 2026-05-31**: `_reset_scene()` removes CO before any planning, re-adds after arm is home |
| Gazebo grey screen with software rendering | OGRE1 FBO render-to-texture fails with LLVMpipe | **Fixed 2026-05-31**: `OGRE_RTT_MODE=Copy` injected automatically when `use_gpu:=false` |
| Gazebo extreme lag without `use_gpu:=true` on WSL2+GPU | Mesa falls back to softpipe instead of D3D12 | Use `use_gpu:=true` to force Mesa D3D12 hardware path |
| STEP 2 fraction < 0.92 | Collision object blocks Cartesian path | Executor auto-tries next grasp candidate |
| Finger gap < 5 mm after close | Grasp pose miss or object out of workspace | Executor auto-tries next candidate |
| Finger gap > 40 mm after close | Object tipped during approach or gripper PID slow to engage | `gripper_read_settle` in YAML controls settle time |
| Camera pointcloud silent after 30 s | Sensor thread still initialising | Wait; bridge auto-reconnects |
| `list_controllers` shows 0 | Controllers not yet spawned (~8 s after gz-sim start) | Wait and retry |
| Spawn error "entity already exists" | Previous `target_obj` still in world | Delete first (see Object Management) |
| Lift fails, object drops | Friction-based lift; DART contact insufficient | Check finger gap > 5 mm; fallback: weld at tag `weld-fallback b374f0e` |
| Return HOME after lift unstable | MoveIt2 planning from post-grasp config occasionally fails | Open issue — observed 2026-05-30; under investigation |

---

## Gate Checklist

| Gate | Test | Status |
|------|------|--------|
| #1 Static hold | Robot spawns in compact-home pose, no oscillation for 10 s | ✅ Confirmed 2026-05-30 |
| #2 Pre-grasp no abort | STEP 1 completes without `GOAL_TOLERANCE_VIOLATED` | ✅ Confirmed 2026-05-30 |
| #3 Grasp + lift | Object lifted by friction (no weld) | ✅ Confirmed 2026-05-30 — 2/2 successful |
| Pick-and-place | Full sequence: grasp + lift + transfer + lower + release + home | ✅ Confirmed 2026-06-01 |
| GSR ≥ 5 runs | Count successes across ≥ 5 single-object attempts | 🔧 In progress |
| Multi-object sweep | scene_manager + execute_grasp target class | 🔧 Implemented, not yet tested |
