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
cd ~/icgnet_ws

# 2. Install ROS package dependencies automatically
rosdep install --from-paths src --ignore-src -y

# 3. Build the workspace
colcon build --symlink-install

# 4. Source the environment
source install/setup.bash
```

> **Tip:** To automatically load the workspace in every new terminal, add this to your `~/.bashrc`:
> `echo "source ~/icgnet_ws/install/setup.bash" >> ~/.bashrc`

### 5. Verification (Smoke Test)
To verify that everything is installed correctly, launch the unified environment (Gazebo + RViz + TF + Robot):

```bash
ros2 launch icgnet_main world.launch.py
```
*You should see Gazebo opening with the Franka Panda robot spawned in front of a table, and RViz showing the PointCloud perfectly aligned with the robot.*

> **Note for RViz:** When you open RViz for the first time, add a `PointCloud2` display, set the topic to `/camera/rgbd_camera/points` (or `/camera/points`), and change the Fixed Frame to `camera_link`. Then go to `File -> Save Config` to make this automatic for future launches.

### 6. Spawning Objects
To test the grasping, you can spawn objects on the table while Gazebo is running. Open a new terminal and choose one of the following methods:

**Method A: Spawn Local Model (Recommended, Instant)**
Spawns a local model without relying on the internet.
```bash
ros2 run gazebo_ros spawn_entity.py -entity my_local_can -file ~/icgnet_ws/src/icgnet_main/models/coke_can/model.sdf -x 0.65 -y 0.0 -z 0.5
```

**Method B: Spawn from Gazebo Online Database (Slower)**
Downloads the model from `models.gazebosim.org`. This might hang for a few minutes on the first run while it downloads.
```bash
ros2 run gazebo_ros spawn_entity.py -database coke_can -entity online_can -x 0.65 -y 0.2 -z 0.5
```
