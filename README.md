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

### 2. Python Dependencies
Install the required Python packages for pointcloud manipulation:
```bash
pip3 install --user open3d numpy scipy
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

### 7. Spawning Objects
To test the grasping, you can spawn objects on the table while Gazebo is running. Open a new terminal and choose one of the following methods:

**Method A: Spawn Local Model (Recommended, Instant)**
Spawns a local model without relying on the internet.
```bash
ros2 run gazebo_ros spawn_entity.py -entity my_local_can -file ~/instance-centric-grasping/src/icgnet_main/models/coke_can/model.sdf -x 0.65 -y 0.0 -z 0.5
```

**Method B: Spawn from Gazebo Online Database (Slower)**
Downloads the model from `models.gazebosim.org`. This might hang for a few minutes on the first run while it downloads.
```bash
ros2 run gazebo_ros spawn_entity.py -database coke_can -entity online_can -x 0.65 -y 0.2 -z 0.5
```

---

## 🧠 8. Local ICGNet Inference

This section explains how to run the ICGNet model locally to obtain grasp poses.

### A. Install Python Deep Learning Stack
Install the specific versions required by ICGNet and the pre-compiled MinkowskiEngine:

```bash
# 1. Install base requirements
pip install -r requirements.txt

# 2. Install MinkowskiEngine (pre-compiled .whl in the root)
pip install MinkowskiEngine-0.5.4-cp312-cp312-linux_x86_64.whl
```

### B. Prepare the Model
1.  **Clone the ICGNet Repository:**
    ```bash
    git clone https://github.com/renezurbruegg/icg_net.git ~/icg_net
    ```
2.  **Download Weights:** Place your `checkpoint.ckpt` and `config.yaml` in a known folder (e.g., `~/icgnet_weights/`).

### C. Run the Grasp Service Node
Launch the node that bridges the camera data with the ICGNet model:

```bash
ros2 run icgnet_main grasp_service_node \
    --ros-args \
    -p config_path:="/home/user/icgnet_weights/config.yaml" \
    -p icgnet_repo_path:="/home/user/icg_net" \
    -p camera_topic:="/camera/rgbd_camera/points"
```

### D. Trigger Grasp Calculation
To compute the grasps for the current scene, call the service:
```bash
ros2 service call /icgnet/compute_grasps std_srvs/srv/Trigger
```
The resulting poses will be published to `/icgnet/grasps` as a `PoseArray`. You can visualize them in RViz by adding a **PoseArray** display.

### E. MoveIt2 Integration
Use the `/icgnet/grasps` topic to feed your planning pipeline. The poses are relative to the camera frame (`camera_depth_optical_frame`).
