# Instance-Centric Grasping

"Instance-Centric Grasping" developed for the "Robotics" course @PoliTo. Tech stack: Python & ROS2 Humble.

## 🚀 Workspace Setup for Teammates

This guide provides step-by-step instructions to set up the ROS2 workspace on your local machine (Ubuntu 22.04 or WSL2) to match the current project state.

### 1. Prerequisites
Ensure your system is up to date and you have ROS2 Humble installed.

```bash
sudo apt update && sudo apt upgrade -y
```

Install the required ROS2 desktop full version and additional controllers:
```bash
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

### 2. Initialize Rosdep

Initialize and update `rosdep` to manage ROS package dependencies:
```bash
sudo rosdep init
rosdep update
```

### 3. Setup and Build the Workspace
Assuming you have cloned this repository, follow these steps to build the workspace. Note that the necessary robot packages (`franka_description`, `panda_ros2_gazebo`) and our custom packages (`icgnet_main`, `icgnet_msgs`) are already included in the `src` folder.

```bash
# 1. Enter the workspace
cd ~/instance-centric-grasping

# 2. Source ROS2 (required before every colcon build)
source /opt/ros/humble/setup.bash

# 3. 

# 4. Build icgnet_msgs FIRST — other packages depend on it
colcon build --symlink-install --packages-select icgnet_msgs
source install/setup.bash

# 5. Build the rest of the workspace
colcon build --symlink-install
source install/setup.bash
```

> **Note (GPU machine only):** Do NOT use `--symlink-install` when building on the machine where `icg_net` is installed. Use `colcon build --packages-select icgnet_msgs icgnet_main panda_ros2_gazebo` instead. The `--symlink-install` flag conflicts with editable installs under setuptools ≥ 64.

> **Tip:** To automatically load the workspace in every new terminal, add this to your `~/.bashrc`:
> `echo "source ~/instance-centric-grasping/install/setup.bash" >> ~/.bashrc`

### 4. Verification
To verify that everything is installed correctly, launch the unified environment (Gazebo + RViz + TF + Robot):

```bash
ros2 launch icgnet_main world.launch.py
```
*You should see Gazebo opening with the Franka Panda robot on a ground plane, and RViz showing the PointCloud perfectly aligned with the robot.*

### 5. MoveIt2 Test

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

# Run the end-to-end test: arm moves to [0.4, 0.0, 0.5] then gripper closes and opens
ros2 run icgnet_main test_move_to_pose
```

You can override the target pose:
```bash
ros2 run icgnet_main test_move_to_pose \
  --ros-args -p position:="[0.3, 0.2, 0.4]" -p quat_xyzw:="[0.0, 0.707, 0.0, 0.707]"
```

> **Note (WSL2):** The Gazebo GUI may not render correctly on WSL2. The simulation still runs headlessly — verify it via `ros2 control list_controllers` and `ros2 topic hz /joint_states` rather than visually.

### 6. Unified Environment & Object Spawning
The environment supports automated, random object spawning during initialization. You can specify the number of objects and the target type. Note: All required models must be downloaded locally first using the provided script.

**Prerequisite:** Download models locally
```bash
python3 scripts/download_gazebo_models.py
```

**Advanced Spawning Options:**
| Parameter | Description | Default |
|-----------|-------------|---------|
| `target_type` | Model name (e.g., `coke_can`, `cricket_ball`) | `cylinder_offline` |
| `num_objects` | Total number of objects (1 to 5) | `1` |

**Case Scenarios:**

**1. Default (Single Offline Cylinder)**
Spawna solo il cilindro locale in posizione casuale.
```bash
ros2 launch icgnet_main world.launch.py
```

**2. Single Target**
Spawna solo una palla da cricket.
```bash
ros2 launch icgnet_main world.launch.py target_type:=cricket_ball num_objects:=1
```

**3. Multi-Object (Target + Random Distractors)**
Spawna 1 target (es. `bowl`) e 4 distrattori scelti a caso tra le classi ICGNet (incluso il cilindro offline).
```bash
ros2 launch icgnet_main world.launch.py target_type:=bowl num_objects:=5
```

**4. Specific Object Spawn (Fixed Position)**
Per debug, spawna un oggetto specifico in posizione fissa (x=0.65, y=0.0).
```bash
ros2 run icgnet_main spawn_one_entity <model_name> <entity_name>
# Esempio:
ros2 run icgnet_main spawn_one_entity wood_cube_10cm cubo_test
```

##  9. Grasp Execution (Full Pipeline)

This section runs the complete pipeline: ICGNet predicts grasps → `grasp_executor` filters them → MoveIt2 moves the arm to pick up the object.

### Prerequisites — Build (once after cloning or after any change)

Install ROS package dependencies automatically

```bash
rosdep install --from-paths src --ignore-src -y
```

```bash
cd ~/instance-centric-grasping
source /opt/ros/humble/setup.bash
colcon build --packages-select icgnet_msgs icgnet_main panda_ros2_gazebo
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

If it is the first time you setup the environment, remember to `export PYTHONPATH=$~/instance-centric-grasping /.venv/lib/python3.10/site-packages:$PYTHONPATH` in the `~/.bashrc` to load the ICGNet model correctly. 

```bash
cd ~/instance-centric-grasping   
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch icgnet_main icgnet_inference.launch.py
# Wait for: "[icgnet_grasp_node]: ICGNet caricato correttamente."
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


