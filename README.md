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
To verify that everything is installed correctly, launch the Panda robot simulation in Gazebo:

```bash
ros2 launch panda_ros2_gazebo gazebo.launch.py
```
*You should see Gazebo opening with the Franka Panda robot spawned in the center, and RViz showing the robot correctly configured without TF errors.*
