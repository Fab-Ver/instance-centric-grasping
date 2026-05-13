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

### 2. Python Dependencies (uv)

Python dependencies are managed by **[uv](https://docs.astral.sh/uv/)** via `pyproject.toml`. Install uv first:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

Then create the virtual environment and install all ML dependencies (Python 3.12 is managed automatically by uv):

```bash
cd ~/instance-centric-grasping
uv sync
```

> **Note:** `uv sync` automatically downloads Python 3.12 if not present, creates `.venv/`, and installs all packages including PyTorch CUDA and MinkowskiEngine from the pre-compiled wheel.

For build tools (cython, needed only if compiling extensions from source):
```bash
uv sync --group build
```

### 3. Initialize Rosdep
Initialize and update `rosdep` to manage ROS package dependencies:
```bash
sudo rosdep init
rosdep update
```

### 4. Setup and Build the Workspace
Assuming you have cloned this repository, follow these steps to build the workspace. Note that the necessary robot packages (`franka_description`, `panda_ros2_gazebo`) and our custom package (`icgnet_main`) are already included in the `src` folder.

```bash
# 1. Enter the workspace
cd ~/instance-centric-grasping

# 2. Install ROS package dependencies automatically
rosdep install --from-paths src --ignore-src -y

# 3. Build the workspace
colcon build --symlink-install

# 4. Source the environment
source install/setup.bash
```

> **Tip:** To automatically load the workspace in every new terminal, add this to your `~/.bashrc`:
> `echo "source ~/instance-centric-grasping/install/setup.bash" >> ~/.bashrc`

### 5. Verification (Smoke Test)
To verify that everything is installed correctly, launch the unified environment (Gazebo + RViz + TF + Robot):

```bash
ros2 launch icgnet_main world.launch.py
```
*You should see Gazebo opening with the Franka Panda robot spawned in front of a table, and RViz showing the PointCloud perfectly aligned with the robot.*

> **Note for RViz:** When you open RViz for the first time, add a `PointCloud2` display, set the topic to `/camera/rgbd_camera/points` (or `/camera/points`), and change the Fixed Frame to `camera_link`. Then go to `File -> Save Config` to make this automatic for future launches.

### 6. MoveIt2 Test

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

### 7. Unified Environment & Object Spawning
The environment now supports automated, random object spawning during initialization. You can specify the number of objects, the target type, and whether to use local or online models.

**Default Launch (Single local can):**
```bash
ros2 launch icgnet_main world.launch.py
```

**Advanced Spawning Options:**
| Parameter | Description | Default |
|-----------|-------------|---------|
| `mode` | `offline` (local model) or `online` (Gazebo DB) | `offline` |
| `target_type` | Model name from Gazebo Database | `coke_can` |
| `num_objects` | Total number of objects (1 to 5) | `1` |

**Examples:**
```bash
# Spawn 1 online coke can
ros2 launch icgnet_main world.launch.py mode:=online

# Spawn 5 objects (Target: beer bottle + 4 random distractors)
ros2 launch icgnet_main world.launch.py target_type:=beer num_objects:=5 mode:=online
```

**Supported ICGNet Categories:**
Lo spawner sceglie i distrattori in base alle classi riconosciute da ICGNet:
- `coke_can` (Classe: **Can**)
- `water_bottle` (Classe: **Bottle**)
- `wood_cube_10cm` (Classe: **Box**)
- `cricket_ball` (Classe: **Ball**)
- `bowl` (Classe: **Other**)

> **Note sui modelli:** 
> - `water_bottle` sostituisce `beer` per avere una vera bottiglia (non una lattina).
> - `hammer`, `cafe_mug` e `power_drill` sono stati rimossi per instabilità fisica o assenza nel database Gazebo.
> - Gli oggetti vengono spawnati in posizioni casuali sul tavolo, garantendo una distanza minima per evitare sovrapposizioni.

## 🧠 8. Local ICGNet Inference

This section explains how to run ICGNet locally to compute grasp predictions from the Gazebo pointcloud and visualize them in RViz.

### A. Install the Deep Learning Stack (once per machine)

All ML dependencies are declared in `pyproject.toml` and installed via **uv** (see Section 2).

```bash
# 1. Install all ML dependencies (PyTorch CUDA, MinkowskiEngine, PyG, open3d, ...)
uv sync

# 2. Clone the ICGNet repository (not a pip package — cloned separately)
git clone https://github.com/renezurbruegg/icg_net.git ~/icg_net

# 3. Clone icg_benchmark and download the checkpoint
git clone https://github.com/renezurbruegg/icg_benchmark.git ~/icg_benchmark
cd ~/icg_benchmark && python scripts/download_data.py
# → checkpoint at: ~/icg_benchmark/data/icgnet/51--0.656/checkpoint.ckpt
```

> **Python version note:** `uv sync` uses Python 3.12 (set in `.python-version`), which matches the pre-compiled MinkowskiEngine wheel (`cp312`). uv downloads Python 3.12 automatically if not present on the system.

> **ROS2 + venv integration:** ROS2 Humble uses Python 3.10. To run `grasp_service_node` with both ROS2 and ML packages accessible, activate the venv first then source ROS2:
> ```bash
> source .venv/bin/activate
> source /opt/ros/humble/setup.bash
> source install/setup.bash
> # ROS2 packages are added to PYTHONPATH on top of the venv
> ```

### B. Configure the Parameters (once)

Edit `src/icgnet_main/config/icgnet_params.yaml` and set the two paths:

```yaml
icgnet_grasp_node:
  ros__parameters:
    config_path:      "~/icg_benchmark/data/icgnet/51--0.656/config.yaml"
    icgnet_repo_path: "~/icg_net"
    # Other parameters (camera_topic, n_grasps, score_threshold) have sensible defaults
```

### C. Run the Full Pipeline

**Terminal 1 — Simulation + RViz:**
```bash
source install/setup.bash
ros2 launch icgnet_main world.launch.py
```
RViz opens automatically with the ICGNet grasp displays pre-loaded (`ICGNet Grasps` MarkerArray).

**Terminal 2 — ICGNet Inference Node:**
```bash
source install/setup.bash
ros2 launch icgnet_main icgnet_inference.launch.py
# Wait for: "ICGNet caricato correttamente." (model loading takes ~10-20s on GPU)
```

**Terminal 3 — Trigger a Prediction:**
```bash
ros2 service call /icgnet/compute_grasps std_srvs/srv/Trigger
# Output: success=True, message="32 grasp pubblicati (32 totali, soglia score>=0.0)"
```

### D. Visualize in RViz

The RViz instance started by `world.launch.py` already has two displays configured:

| Display | Topic | Description |
|---------|-------|-------------|
| **ICGNet Grasps** | `/icgnet/grasps_markers` | Arrows colored by score: 🟢 green = high quality, 🔴 red = low |
| ICGNet PoseArray | `/icgnet/grasps` | Standard ROS pose axes (disabled by default) |

Each arrow points along the gripper approach direction. Grasp centers are placed on the detected objects.

> **Tip:** Increase `score_threshold` in `icgnet_params.yaml` (e.g. `0.5`) to show only the best grasps and reduce visual clutter.

### E. Architecture & Topics

```
Gazebo Camera
  └─ /camera/rgbd_camera/points  (PointCloud2, BEST_EFFORT)
       │
       ▼
  grasp_service_node
  ├─ Transforms cloud: camera_link_optical → world (via tf2)
  ├─ Preprocesses: voxel downsample + normal estimation
  ├─ Runs ICGNet inference
  └─ Publishes:
       ├─ /icgnet/grasps          (PoseArray, frame=world)
       └─ /icgnet/grasps_markers  (MarkerArray, arrows colored by score)
```

Grasp poses are published in the **`world` frame** (table at z=0, as required by ICGNet).
The future `grasp_executor` will only need to transform `world → panda_link0` before sending to MoveIt2.

### F. Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| `"Nessuna pointcloud ricevuta"` | Gazebo not running or wrong topic | Check `ros2 topic hz /camera/rgbd_camera/points` |
| `"TF lookup fallito"` | `world.launch.py` not running | Launch simulation first |
| `"ICGNet non inizializzato"` | Wrong paths in YAML | Check `config_path` and `icgnet_repo_path` |
| Arrows appear at wrong location | Fixed Frame mismatch | Set RViz Fixed Frame to `world` |
| No arrows after trigger | All grasps below threshold | Set `score_threshold: 0.0` in YAML |
