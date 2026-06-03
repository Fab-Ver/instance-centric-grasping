# Testing Guide — ICGNet ROS2 Gazebo Sim Fortress

**Branch:** `main`
**Stack:** ROS2 Humble + Gazebo Sim Fortress (gz-sim 6, DART physics) + MoveIt2 + gz_ros2_control
**Last updated:** 2026-06-03

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

**With GPU (WSL2 + NVIDIA):**

```bash
cd ~/instance-centric-grasping
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch icgnet_main world.launch.py use_gpu:=true
```

Wait for:
- Gazebo GUI window opens (OGRE1 renderer)
- `[controller_manager]` ready
- `[move_group]` → `Ready to take commands for planning group arm.`
- `[gz_ros_bridge]` publishing (camera sensor init takes ~10 s)

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

# Spawns 2 cans (target_obj_0, target_obj_1) + 2–3 random distractors of OTHER classes.
# Node stays alive to serve /icgnet/reset_scene.
ros2 run icgnet_main scene_manager --ros-args \
  -p target_class:=can -p target_count:=2

# Verify manifest published:
ros2 topic echo /icgnet/scene_manifest --once

# Manual reset (teleports all objects back to spawn pose):
ros2 service call /icgnet/reset_scene std_srvs/srv/Trigger
```

Available classes: `can`, `bottle`, `box`, `mug`, `cylindric`, `ball`, `other`

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

# Step 2: execute
# Single-object: grasps one object and places it in the bin
# Multi-object: sweeps all instances of the target class into the bin, distractors stay
ros2 service call /icgnet/execute_grasp icgnet_msgs/srv/ExecuteGrasp "{target: 'can'}"
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
