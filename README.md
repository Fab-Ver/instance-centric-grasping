# Instance-Centric Grasping

ICGNet-based robotic grasping in simulation, developed for the *Robotics* course @PoliTo.

**Stack:** ROS2 Humble · Gazebo Sim Fortress (gz-sim 6, DART physics) · MoveIt2 · `gz_ros2_control` · ICGNet (GPU inference).

> This guide assumes an **NVIDIA GPU** (required for ICGNet inference).

---

## 1. Prerequisites (one-time setup)

### 1.1 System dependencies (apt)

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

`rosdep` resolves the rest from the package manifests:

```bash
sudo rosdep init && rosdep update          # first time only
cd ~/instance-centric-grasping
rosdep install --from-paths src --ignore-src -y
```

### 1.2 Python ML stack (ICGNet inference)

Dependencies are managed by **uv** via `pyproject.toml` (Python 3.12 — torch, MinkowskiEngine, PyG, trimesh, …):

```bash
cd ~/instance-centric-grasping
uv sync                  # creates .venv with all ML deps
uv sync --group build    # optional: build tools (cython) for compiling extensions from source
```

### 1.3 ICGNet model

The network is loaded at runtime from a cloned `icg_net` repository added to `sys.path` (it is **not** a pip dependency).

```bash
# 1. Clone the icg_net repo next to the workspace
git clone <icg_net-repo-url> ~/icg_net

# 2. Apply the bundled patches (Hydra config loading + grasp decoding)
cp scripts/patches/icg_net.py  ~/icg_net/icg_net/icg_net.py
cp scripts/patches/grasps.py   ~/icg_net/icg_net/utils/grasps.py
```

The checkpoint is bundled in `icgnet_weights/` (`checkpoint.ckpt` + `config.yaml`). Point the node to it in
`src/icgnet_main/config/icgnet_params.yaml`:

```yaml
icgnet_grasp_node:
  ros__parameters:
    config_path:      "<abs-path-to-repo>/icgnet_weights/config.yaml"
    icgnet_repo_path: "~/icg_net"
```

### 1.4 Download Gazebo object models

```bash
python3 scripts/download_gazebo_models.py
```

---

## 2. Build

Every new shell must source the environment before any `ros2` command:

```bash
source /opt/ros/humble/setup.bash
source ~/instance-centric-grasping/install/setup.bash
```

Build from the workspace root:

```bash
cd ~/instance-centric-grasping
colcon build --packages-select panda_description icgnet_msgs icgnet_main pymoveit2
source install/setup.bash
```

---

## 3. Running the pipeline

Five terminals. All start from `~/instance-centric-grasping` and source the environment first.
ICGNet predicts grasps → `grasp_executor` filters them → MoveIt2 moves the arm to pick up the object.

### T1 — Simulation + MoveIt2 + camera bridge + TF

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch icgnet_main world.launch.py
```

> **Headless gz (default `headless:=true`)**: no gz GUI window — RViz is the only visualization, with object

Wait for:
- `[controller_manager]` ready
- `[move_group]` → `Ready to take commands for planning group arm.`
- `[gz_ros_bridge]` publishing (camera sensor init takes ~10 s)
- RViz opens; `[scene_visualizer]` → `SceneVisualizerNode ready`

### T2 — Spawn objects (after T1 ready, ~15 s)

Two modes — pick one. Available classes: `can`, `box`, `mug`, `ball`, `bottle`, `cylindric`.

**A — Single object:**

```bash
ros2 run icgnet_main spawn_object --ros-args -p target_class:=can
# or by exact model: -p target_type:=coke_can
```

**B — Multi-object scene (`scene_manager`):**

```bash
# Spawns target-class objects + random distractors of OTHER classes.
# target_class / target_count only control spawn quantities — execute_grasp can target ANY
# class present in the scene. Node stays alive to serve /icgnet/reset_scene.
ros2 run icgnet_main scene_manager --ros-args -p target_class:=can -p target_count:=2

# Inspect the scene:
ros2 topic echo /icgnet/scene_manifest --once          # all objects + semantic class
ros2 service call /icgnet/reset_scene std_srvs/srv/Trigger   # teleport all back to spawn pose
```

### T3 — ICGNet inference (GPU)

> colcon executables have a hardcoded `/usr/bin/python3` shebang, so `source .venv/bin/activate` is **not**
> enough — export `PYTHONPATH` from the repo root so the node finds the ML stack.

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
export PYTHONPATH=$(pwd)/.venv/lib/python3.12/site-packages:$PYTHONPATH
ros2 launch icgnet_main icgnet_inference.launch.py
# Expected: "ICGNet loaded successfully." + "ICGNetGraspNode ready"  (model load ~10-20 s)
```

### T4 — Grasp executor

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch icgnet_main grasp_execution.launch.py
# Expected: "GraspExecutorNode ready"
```

### T5 — Triggers (after T1–T4 are up)

```bash
# Step 1: run inference (grasp_executor also triggers this internally on each execute_grasp)
ros2 service call /icgnet/compute_grasps std_srvs/srv/Trigger

# Step 2: execute — `target` sets WHAT to grasp, independent of how objects were spawned.
#   single object (T2-A): grasps it if it matches the target class.
#   multi-object  (T2-B): sweeps ALL instances of the target class into the bin; other classes
#                         are left in place. An absent class returns an error listing present ones.
ros2 service call /icgnet/execute_grasp icgnet_msgs/srv/ExecuteGrasp "{target: 'can'}"
ros2 service call /icgnet/execute_grasp icgnet_msgs/srv/ExecuteGrasp "{target: 'box'}"
ros2 service call /icgnet/execute_grasp icgnet_msgs/srv/ExecuteGrasp "{target: 'any'}"

# Debug (lift only, no place):
ros2 service call /icgnet/execute_grasp icgnet_msgs/srv/ExecuteGrasp "{target: 'can', skip_place: true}"
```

Grasp arrows appear in RViz on `/icgnet/grasps_markers` (green = high score, red = low score).

### Expected grasp log sequence

```
[SPAWN_POSE] Object spawn pose received: (0.65, 0.00, 0.052)
[INSTANCES] 1 instance(s): inst_0=can(id=2, 353g) | total_grasps=353
[FILTER]  total=353 → kept=353 (scores=[0.30–0.81]) | rejected: width=0 workspace=0 target=0
[PLAN]    score=0.807  inst=0  cls=2(can)  width=0.0660m
[STEP 1/5] PRE-GRASP  → [x, y, z+0.12]
[STEP 2/5] APPROACH   → [x, y, z]
[STEP 3/5] CLOSING GRIPPER  gap=21.3mm  ✓
[STEP 4/5] LIFTING    → [x, y, z+0.18]
[STEP 5/7] TRANSFER   → (0.45, -0.50, 0.35)
[STEP 6/7] LOWER      → z=0.26
[STEP 7/7] OPEN GRIPPER + settle + retract + HOME
[SUCCESS]  Grasp completed on attempt 1/5
```

---

## 4. Headless verification (no GUI required)

After T1 is up, in a separate shell:

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash

# 3 controllers must be active:
ros2 control list_controllers
# joint_state_broadcaster   [.../JointStateBroadcaster]            active
# panda_arm_controller      [.../JointTrajectoryController]        active
# panda_hand_controller     [.../JointTrajectoryController]        active

ros2 node list | grep move_group                    # MoveIt2 move_group up
ros2 action list | grep follow_joint_trajectory     # FollowJointTrajectory actions
ros2 topic hz /camera/rgbd_camera/points            # camera pointcloud (~30 Hz, after ~10 s)

# After a compute_grasps trigger:
ros2 topic echo /icgnet/grasps_rich --once          # ICGNet predictions
ros2 topic hz /icgnet/scene_meshes                  # RViz scene mirror (~10 Hz)
ros2 run tf2_tools view_frames                      # world → <entity>_viz frames
```

In RViz the object mesh stays **locked to the gripper** during approach/lift/transport — each object gets its
own `<entity>_viz` TF frame and a `frame_locked` marker.

---

## 5. Object management

```bash
# Delete an object from gz-sim:
ros2 service call /world/icgnet_world/remove ros_gz_interfaces/srv/DeleteEntity \
  "{entity: {name: 'target_obj', type: 2}}"

# Teleport an object back to its init position:
ros2 service call /world/icgnet_world/set_pose ros_gz_interfaces/srv/SetEntityPose \
  "{entity: {name: 'target_obj', type: 2}, pose: {position: {x: 0.65, y: 0.0, z: 0.05}}}"
```

---

## 6. Phase 1 evaluation — automated single-object benchmark

`scripts/run_evaluation_phase1.py` drives the full grasp pipeline automatically across many runs, one object
class at a time. It spawns one object, calls `/icgnet/execute_grasp`, and logs the
outcome plus the per-attempt failure reason.

**Prerequisites:** bring up **T1 + T3 + T4** (the script handles spawn + grasp itself — no T2/T5). Launch T1
with `rviz:=false` for a faster headless batch. `/model_poses` must be publishing — it is the ground truth used
to decide success:

```bash
ros2 topic echo /model_poses --once   # must print a TFMessage
```

**Run:**

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash

# Default: 6 classes × 20 runs, target-driven
./scripts/run_evaluation_phase1.py

# Quick subset / class-agnostic mode:
./scripts/run_evaluation_phase1.py --runs-per-class 30 --classes can ball
./scripts/run_evaluation_phase1.py --runs-per-class 30 --mode any
```

| Option | Default | Meaning |
|--------|---------|---------|
| `--runs-per-class N` | `20` | runs per object class |
| `--classes ...` | all 6 catalog classes | which classes to evaluate |
| `--mode {target,any}` | `target` | `target` = grasp the spawned class (needs correct ICGNet classification); `any` = class-agnostic. The two modes write to separate files. |

**Output** — versioned under `results/tests/` (never overwrites): `eval_<...>_v<N>.csv` (one row per run),
`eval_<...>_v<N>_summary.txt` (aggregated GSR + failure histograms), `inference_<...>_v<N>.jsonl` (full
pre-filter ICGNet proposal set) and `grasping_<...>_v<N>.jsonl` (per-attempt geometry + outcome). The CSV is
flushed every run, so it stays usable if the batch is interrupted.

**Failure-mode codes** (`Failure_Reason` / `Attempt_Reasons`): `SUCCESS`, `PERCEPTION_NO_GRASP`,
`PREGRASP_PLAN_FAIL`, `APPROACH_FAIL`, `GRASP_MISS`, `OBJECT_TIPPED`, `LIFT_PLAN_FAIL`, `LIFT_DROP`,
`TRANSFER_PLAN_FAIL`, `TRANSFER_DROP`, `LOWER_PLAN_FAIL`, `LOWER_DROP`, `PLACE_ROLLOUT`, `SPAWN_FAIL`.

