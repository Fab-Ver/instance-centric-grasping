# Instance-Centric Grasping

"Instance-Centric Grasping" developed for the "Robotics" course @PoliTo. Tech stack: Python & ROS2 Humble.

---

## Prerequisites (one-time setup)

### 1. System dependencies

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y \
    ros-humble-desktop-full \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-moveit \
    ros-humble-moveit-resources \
    ros-humble-controller-manager \
    ros-humble-joint-state-publisher-gui \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gripper-controllers \
    ros-humble-joint-state-broadcaster \
    python3-colcon-common-extensions \
    python3-rosdep python3-vcstool
```

### 2. Initialize rosdep

```bash
sudo rosdep init
rosdep update
```

### 3. Build the workspace

```bash
cd ~/instance-centric-grasping
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select icgnet_msgs icgnet_main panda_description
source install/setup.bash
```

### 4. Download Gazebo models

Required to spawn objects from the Gazebo online model database:

```bash
python3 scripts/download_gazebo_models.py
```

---

## Grasp Execution Pipeline

This section runs the complete pipeline: ICGNet predicts grasps → `grasp_executor` filters them → MoveIt2 moves the arm to pick up the object.

### Prerequisites — Build (once after cloning or after any change)

Install ROS package dependencies automatically

```bash
rosdep install --from-paths src --ignore-src -y
```

```bash
cd ~/instance-centric-grasping
source /opt/ros/humble/setup.bash
colcon build --packages-select icgnet_msgs icgnet_main panda_description
source install/setup.bash
```

Add other packages after `--packages-select` if you made changes to them.

### Terminal 1 — Simulation (Gazebo + MoveIt2 + RViz)

```bash
cd ~/instance-centric-grasping
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch icgnet_main world.launch.py
# Wait until you see:
#   [move_group]: MoveGroup context using planning plugin ...
#   [move_group]: All is well! Everyone is happy!
```

### Terminal 2 — ICGNet Inference Node (GPU machine)

If it is the first time you setup the environment, remember to add `export PYTHONPATH=$(pwd)/.venv/lib/python3.10/site-packages:$PYTHONPATH` to your `~/.bashrc` (run from the repo root) to load the ICGNet model correctly.

```bash
cd ~/instance-centric-grasping
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch icgnet_main icgnet_inference.launch.py
# Wait for: "[icgnet_grasp_node]: ICGNet loaded successfully."
# Model loading takes ~10-20s on GPU.
```

### Terminal 3 — Grasp Executor

```bash
cd ~/instance-centric-grasping
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch icgnet_main grasp_execution.launch.py
# Wait for: "[grasp_executor_node]: GraspExecutorNode ready."
```

### Terminal 4 — Trigger a Grasp

```bash
cd ~/instance-centric-grasping
source /opt/ros/humble/setup.bash && source install/setup.bash

# Baseline: pick the best grasp regardless of object type
ros2 service call /icgnet/execute_grasp icgnet_msgs/srv/ExecuteGrasp \
  "{target: 'any', min_score: 0.4, max_attempts: 5}"

# Target-driven: pick only a can
ros2 service call /icgnet/execute_grasp icgnet_msgs/srv/ExecuteGrasp \
  "{target: 'can', min_score: 0.4, max_attempts: 5}"

# Target by instance ID (useful for multi-object scenes)
ros2 service call /icgnet/execute_grasp icgnet_msgs/srv/ExecuteGrasp \
  "{target: 'instance_0', min_score: 0.3, max_attempts: 3}"
```

**Expected response on success:**
```
success: True
grasps_attempted: 2
message: Grasp succeeded on attempt 2
```

**Expected response when no matching object:**
```
success: False
grasps_attempted: 0
message: "No grasps after filtering: target='mug' min_score=0.40"
```

---

## Local ICGNet Inference

This section explains how to run ICGNet on the GPU machine to compute grasp poses from the Gazebo pointcloud and visualize them in RViz.

### A. Deep Learning stack setup (once per machine)

Full step-by-step guide: `LOCAL_INFERENCE_SETUP.md`

Key points:
- **Python 3.10** (system) + standard `venv` + `pip` — do NOT use `uv`
- MinkowskiEngine must be compiled from source (fork `renezurbruegg/MinkowskiEngine`). Set `TORCH_CUDA_ARCH_LIST` to your GPU only (e.g. `"8.6"` for RTX 30xx, `"6.1"` for GTX 10xx) and `MAX_JOBS=2` to avoid OOM during compilation
- `numpy==1.26.4` — numpy >= 2.0 breaks PyTorch 2.2.0
- Install `icg_net` with `python setup.py develop` — `pip install -e` fails with setuptools >= 67
- Copy the Hydra patch: `cp scripts/patches/icg_net.py ~/icg_net/icg_net/icg_net.py`

### B. Configure parameters (once)

Edit `src/icgnet_main/config/icgnet_params.yaml` with the correct paths for your machine:

```yaml
icgnet_grasp_node:
  ros__parameters:
    config_path:      "~/icg_benchmark/data/icgnet/51--0.656/config.yaml"
    icgnet_repo_path: "~/icg_net"
```

### C. Run the inference node

**Terminal 1 — Simulation (if not already running):**
```bash
source setup_ws.sh
ros2 launch icgnet_main world.launch.py
```

**Terminal 2 — ICGNet node (GPU machine):**
```bash
cd ~/instance-centric-grasping
source /opt/ros/humble/setup.bash && source install/setup.bash
export PYTHONPATH=$(pwd)/.venv/lib/python3.10/site-packages:$PYTHONPATH
ros2 launch icgnet_main icgnet_inference.launch.py
# Wait for: "[icgnet_grasp_node]: ICGNet loaded successfully."
# Model loading takes ~10-20s on GPU.
```

> **Note:** `source .venv/bin/activate` is NOT sufficient — colcon executables have a hardcoded `/usr/bin/python3` shebang. Always `export PYTHONPATH=...` from the repo root.

**Terminal 3 — Trigger inference:**
```bash
source setup_ws.sh
ros2 service call /icgnet/compute_grasps std_srvs/srv/Trigger
# Expected: success=True, message="Published N grasps (M total, score>=0.0)"
```

Grasp arrows appear in RViz on `/icgnet/grasps_markers` (green = high score, red = low score).

## MoveIt2 Test

Once the simulation is running, verify the MoveIt2 pipeline is fully operational from a second terminal:

```bash
source install/setup.bash

# Check that both trajectory controllers are active
ros2 control list_controllers
# Expected output:
# joint_state_broadcaster  joint_state_broadcaster/JointStateBroadcaster          active
# panda_hand_controller    joint_trajectory_controller/JointTrajectoryController  active
# panda_arm_controller     joint_trajectory_controller/JointTrajectoryController  active

# Check that the FollowJointTrajectory actions are available
ros2 action list | grep follow
# Expected output:
# /panda_arm_controller/follow_joint_trajectory
# /panda_hand_controller/follow_joint_trajectory

# End-to-end test: arm moves to [0.4, 0.0, 0.5], then gripper opens and closes
ros2 run icgnet_main test_move_to_pose
```

> **Note (WSL2):** The Gazebo GUI may not render correctly on WSL2. The simulation still runs headlessly — verify via `ros2 control list_controllers` and `ros2 topic hz /joint_states`.
