# Testing Guide — ICGNet ROS2 Gazebo Sim Fortress

**Branch:** `improved_version_to_test`
**Stack:** ROS2 Humble + Gazebo Sim Fortress (gz-sim 6, DART physics) + MoveIt2 + gz_ros2_control
**Last updated:** 2026-05-30

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

## Terminal Sequence

All terminals: start from `~/instance-centric-grasping` and source the environment first.

### T1 — Simulation + MoveIt2 + Camera bridge + TF

```bash
cd ~/instance-centric-grasping
source /opt/ros/humble/setup.bash && source install/setup.bash
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch icgnet_main world.launch.py
```

Wait for:
- gz-sim started (GUI opens on native Linux; headless on WSL2)
- `[controller_manager]` ready
- `[move_group]` `Ready to take commands for planning group arm.`
- `[gz_ros_bridge]` publishing (camera sensor init takes ~10 s)

### T2 — Spawn object (after T1 ready, ~15 s)

```bash
cd ~/instance-centric-grasping
source /opt/ros/humble/setup.bash && source install/setup.bash

# By semantic class (picks random model from catalog):
ros2 run icgnet_main spawn_object --ros-args -p target_class:=can
ros2 run icgnet_main spawn_object --ros-args -p target_class:=bottle
ros2 run icgnet_main spawn_object --ros-args -p target_class:=box

# By exact model name:
ros2 run icgnet_main spawn_object --ros-args -p target_type:=beer_can
ros2 run icgnet_main spawn_object --ros-args -p target_type:=coke_can

# Low-level gz-sim spawn (absolute path):
PKG=$(ros2 pkg prefix icgnet_main)/share/icgnet_main
ros2 run ros_gz_sim create \
  -world icgnet_world -name target_obj \
  -file $PKG/models/can/beer_can/model.sdf \
  -x 0.65 -y 0.0 -z 0.05
```

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

> Prerequisite: generate `~/icgnet_inference_data/` first (see Save Inference section).

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
# Step 1: run inference (or replay saved data)
ros2 service call /icgnet/compute_grasps std_srvs/srv/Trigger

# Step 2: execute grasp
ros2 service call /icgnet/execute_grasp icgnet_msgs/srv/ExecuteGrasp "{target: 'any'}"
# or target a semantic class:
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 service call /icgnet/execute_grasp icgnet_msgs/srv/ExecuteGrasp "{target: 'can'}"
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
# Delete object from gz-sim (gz-native, NOT gazebo_msgs/DeleteEntity):
ros2 service call /world/icgnet_world/remove ros_gz_interfaces/srv/DeleteEntity \
  "{entity: {name: 'target_obj', type: 2}}"

# Teleport object back to init position (grasp_executor does this automatically on reset):
ros2 service call /world/icgnet_world/set_pose ros_gz_interfaces/srv/SetEntityPose \
  "{entity: {name: 'target_obj', type: 2}, pose: {position: {x: 0.65, y: 0.0, z: 0.05}}}"
```

---

## Expected Grasp Log Sequence

```
[FILTER]  N grasps pass score>=0.40  [0.41–0.87]
[PLAN]    score=0.752  inst=0  cls=2  width=0.0712m
[PRE-0]   gripper → pre-grasp width=35.6mm  (ICGNet=71.2mm/2 + 10mm)
[STEP 1/5] PRE-GRASP  → [x, y, z-0.12]
[STEP 2/5] APPROACH   → [x, y, z]   fraction=1.00
[STEP 3/5] CLOSING GRIPPER  gap=21.3mm  ✓
[STEP 3b]  Attached 'icgnet_inst_0' to panda_hand_tcp
[STEP 4/5] LIFTING    → [x, y, z+0.25]
[STEP 5/5] HOME
[SUCCESS]  Grasp completed on attempt 1/5
```

---

## Known Issues

| Symptom | Cause | Fix |
|---|---|---|
| STEP 2 fraction < 0.92 | Collision object blocks Cartesian path | Executor auto-tries next grasp candidate |
| Finger gap < 5 mm after close | Grasp pose miss or object out of workspace | Executor auto-tries next candidate |
| Camera pointcloud silent after 30 s | Sensor thread still init | Wait; bridge auto-reconnects |
| `list_controllers` shows 0 | Controllers not yet spawned (~8 s after gz-sim start) | Wait and retry |
| Spawn error "entity already exists" | Previous target_obj still in world | Delete first (see Object Management) |
| Lift fails, object drops | Friction-based lift; DART contact may be insufficient | Check finger gap > 5 mm; if consistently fails, see weld fallback at tag `weld-fallback b374f0e` |

---

## Gate Checklist

| Gate | Test | Status |
|------|------|--------|
| #1 Static hold | Robot spawns in compact-home pose, no oscillation for 10 s | ❌ Run T1, check `ros2 topic echo /joint_states` |
| #2 Pre-grasp no abort | STEP 1 completes without `GOAL_TOLERANCE_VIOLATED` | ❌ Run full T1–T5 |
| #3 Grasp + lift | Object lifted ≥ 0.25 m by friction (no weld) | ❌ Blocked on gate #2 |
| GSR ≥ 5 runs | Count successes across 5 single-object attempts | ❌ Blocked on gate #2 |
