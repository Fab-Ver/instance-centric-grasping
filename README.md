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

# 3. Install ROS package dependencies automatically
rosdep install --from-paths src --ignore-src -y

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

### 4. Verification (Smoke Test)
To verify that everything is installed correctly, launch the unified environment (Gazebo + RViz + TF + Robot):

```bash
ros2 launch icgnet_main world.launch.py
```
*You should see Gazebo opening with the Franka Panda robot on a ground plane (no table), and RViz showing the PointCloud perfectly aligned with the robot.*

> **Note for RViz:** When you open RViz for the first time, add a `PointCloud2` display, set the topic to `/camera/rgbd_camera/points` (or `/camera/points`), and change the Fixed Frame to `camera_link`. Then go to `File -> Save Config` to make this automatic for future launches.

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

**Supported ICGNet Categories:**
Lo spawner seleziona i distrattori in base alle classi riconosciute da ICGNet:
- `coke_can` (Classe: **Can**)
- `wood_cube_10cm` (Classe: **Box**)
- `cricket_ball` (Classe: **Ball**)
- `bowl` (Classe: **Mug/Other**)
- `cylinder_offline` (Classe: **Cylindric**)
- `monkey_wrench` (Classe: **Other**)

> **Note:**
> - Gli oggetti vengono spawnati in posizioni casuali sul ground (z=0.05), garantendo una distanza minima per evitare sovrapposizioni.
> - Se `num_objects > 1`, i distrattori vengono scelti casualmente dalla lista sopra.

## 🧠 8. Local ICGNet Inference

This section explains how to run ICGNet locally to compute grasp predictions from the Gazebo pointcloud and visualize them in RViz.

### A. Install the Deep Learning Stack (once per machine)

**Full step-by-step guide**: `LOCAL_INFERENCE_SETUP.md` in the repo root.

Key points:
- Uses **Python 3.10** (system) + standard `venv` + `pip`.
- MinkowskiEngine must be **compiled from source** from the patched fork (`renezurbruegg/MinkowskiEngine`). Set `TORCH_CUDA_ARCH_LIST` to your GPU's compute capability only (e.g. `"8.6"` for RTX 30xx, `"6.1"` for GTX 10xx) to avoid OOM.
- `icg_net` cannot be installed with `pip install -e` (setuptools ≥ 67 bug). Use `python setup.py develop` or a `.pth` file. A patched `icg_net/icg_net/icg_net.py` is in `scripts/patches/` — copy it over before running.
- **numpy must be `1.26.4`** (not ≥2.0) for compatibility with PyTorch 2.2.0.

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
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch icgnet_main world.launch.py
```
RViz opens automatically with the ICGNet grasp displays pre-loaded (`ICGNet Grasps` MarkerArray).

**Terminal 2 — ICGNet Inference Node:**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
# Use $(pwd) to get the correct absolute path regardless of where the repo is cloned
export PYTHONPATH=$(pwd)/.venv/lib/python3.10/site-packages:$PYTHONPATH
ros2 launch icgnet_main icgnet_inference.launch.py
# Wait for: "ICGNet caricato correttamente." (model loading takes ~10-20s on GPU)
```

> **Note:** `source .venv/bin/activate` is NOT sufficient — the installed executable has a hardcoded shebang pointing to the system Python. `PYTHONPATH` is the correct mechanism to expose the venv's ML packages to the system Python 3.10. Run `export PYTHONPATH=...` from inside the repo root (where `.venv/` is located).

**Terminal 3 — Trigger a Prediction:**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 service call /icgnet/compute_grasps std_srvs/srv/Trigger
# Output: success=True, message="N grasp pubblicati (N totali, soglia score>=0.0)"
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
  grasp_service_node  (icgnet_inference.launch.py)
  ├─ Transforms cloud: camera_link_optical → world (via tf2)
  ├─ Preprocesses: voxel downsample + normal estimation
  ├─ Runs ICGNet inference
  └─ Publishes:
       ├─ /icgnet/grasps          (PoseArray, frame=world)
       ├─ /icgnet/grasps_markers  (MarkerArray, arrows colored by score — RViz)
       └─ /icgnet/grasps_rich     (GraspArray: pose+score+width+instance_id+semantic_class)
                                        │
                                        ▼
                              grasp_executor  (grasp_execution.launch.py)
                              ├─ Filters: score ≥ threshold, width ≤ 0.08m, workspace bounds, target class
                              ├─ Sorts by score (best first)
                              └─ For each grasp:
                                   open gripper → pre-grasp (+10cm) → approach → close → lift (+25cm)
                                        │
                                        ▼
                                   MoveIt2 / pymoveit2
```

Grasp poses are published in the **`world` frame** (table at z=0, as required by ICGNet). The `grasp_executor` does not need an additional TF transform — `world` and `panda_link0` are coincident in Gazebo.

### F. Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| `"Nessuna pointcloud ricevuta"` | Gazebo not running or wrong topic | Check `ros2 topic hz /camera/rgbd_camera/points` |
| `"TF lookup fallito"` | `world.launch.py` not running | Launch simulation first |
| `"ICGNet non inizializzato"` | Wrong paths in YAML | Check `config_path` and `icgnet_repo_path` in `icgnet_params.yaml` |
| `ModuleNotFoundError: torch` at launch | PYTHONPATH not set correctly | Run `export PYTHONPATH=$(pwd)/.venv/lib/python3.10/site-packages:$PYTHONPATH` from repo root |
| `ModuleNotFoundError: torch` despite PYTHONPATH | Wrong working directory when exporting | Make sure you are in the repo root (`cd ~/…/instance-centric-grasping`) before exporting |
| `Cannot find primary config '...'` | hydra.experimental bug | Copy `scripts/patches/icg_net.py` → `~/icg_net/icg_net/icg_net.py` |
| ME compile killed (OOM) | Too many CUDA architectures | Set `TORCH_CUDA_ARCH_LIST` to your GPU only (e.g. `"6.1"`) and `MAX_JOBS=2` |
| Arrows appear at wrong location | Fixed Frame mismatch | Set RViz Fixed Frame to `world` |
| No arrows after trigger | All grasps below threshold | Set `score_threshold: 0.0` in `icgnet_params.yaml` |
| `No module named 'icgnet_msgs'` at executor launch | `icgnet_msgs` not built or not sourced | `colcon build --packages-select icgnet_msgs && source install/setup.bash` |
| `Service '/icgnet/compute_grasps' not available` | `icgnet_inference.launch.py` not running | Start Terminal 3 of section 9 first |
| `No grasps after filtering` | All grasps outside workspace or below score | Lower `default_min_score` in `grasp_executor_params.yaml` or check workspace bounds |
| Arm doesn't move on `execute_grasp` call | MoveIt2 not ready or IK failure | Check `ros2 action list | grep follow`; try `target='any'` with `min_score: 0.0` |

---

## 🤖 9. Grasp Execution (Full Pipeline)

This section runs the complete pipeline: ICGNet predicts grasps → `grasp_executor` filters them → MoveIt2 moves the arm to pick up the object.

> ⚠️ **Status:** `grasp_executor` and `icgnet_msgs` are implemented but **not yet tested end-to-end**. Expect iteration.

### Prerequisites — Build (once after cloning or after any change)

```bash
cd ~/instance-centric-grasping
source /opt/ros/humble/setup.bash

# Build icgnet_msgs first (ament_cmake — other packages depend on it)
colcon build --packages-select icgnet_msgs
source install/setup.bash

# Build icgnet_main (NO --symlink-install on the GPU machine)
colcon build --packages-select icgnet_main
source install/setup.bash

# Verify interfaces were generated correctly
ros2 interface show icgnet_msgs/msg/Grasp
ros2 interface show icgnet_msgs/srv/ExecuteGrasp
```

### Terminal 1 — Simulation (Gazebo + MoveIt2 + RViz)

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch icgnet_main world.launch.py
# Wait until you see:
#   [move_group]: MoveGroup context using planning plugin ...
#   [move_group]: All is well! Everyone is happy!
```

Optionally spawn an object (from a second shell, same terminal window):
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run gazebo_ros spawn_entity.py -entity coke_can \
  -file src/icgnet_main/models/coke_can/model.sdf -x 0.65 -y 0.0 -z 0.05
```

Or use the unified spawner:
```bash
ros2 launch icgnet_main world.launch.py target_type:=coke_can num_objects:=1
```

### Terminal 2 — Verify MoveIt2 is ready

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash

# Both controllers must be active before proceeding
ros2 control list_controllers
# panda_arm_controller  → active
# panda_hand_controller → active

ros2 action list | grep follow
# /panda_arm_controller/follow_joint_trajectory
# /panda_hand_controller/follow_joint_trajectory
```

### Terminal 3 — ICGNet Inference Node (GPU machine)

```bash
cd ~/instance-centric-grasping    # must be in repo root for $(pwd) to resolve correctly
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONPATH=$(pwd)/.venv/lib/python3.10/site-packages:$PYTHONPATH
ros2 launch icgnet_main icgnet_inference.launch.py
# Wait for: "[icgnet_grasp_node]: ICGNet caricato correttamente."
# Model loading takes ~10-20s on GPU.
```

### Terminal 4 — Grasp Executor

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch icgnet_main grasp_execution.launch.py
# Wait for: "[grasp_executor_node]: GraspExecutorNode ready."
```

### Terminal 5 — Trigger a Grasp

```bash
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

### Supported target values

| `target` value | Meaning |
|---|---|
| `any` | Best grasp regardless of class |
| `mug` | Semantic class 0 |
| `box` | Semantic class 1 |
| `can` | Semantic class 2 |
| `bottle` | Semantic class 3 |
| `cylindric` | Semantic class 4 |
| `ball` | Semantic class 5 |
| `other` | Semantic class 6 |
| `instance_N` | Specific instance ID (e.g. `instance_0`) |

### Tunable parameters

Edit `src/icgnet_main/config/grasp_executor_params.yaml` before rebuilding:

| Parameter | Default | Effect |
|---|---|---|
| `default_min_score` | `0.4` | Lower if too many grasps are discarded |
| `approach_offset` | `0.10` m | Distance of pre-grasp from object. Increase if robot hits object on approach |
| `lift_height` | `0.25` m | How far the arm rises after grasping |
| `workspace_x/y/z_min/max` | see yaml | Franka reachable workspace in world frame. Widen if valid grasps are discarded |
