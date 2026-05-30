# panda_description

Franka Panda URDF + Gazebo Sim Fortress (gz-sim 6) bringup for the ICGNet grasping project.

## Stack

- ROS2 Humble
- Gazebo Sim Fortress (gz-sim 6, DART physics)
- gz_ros2_control — arm: `position` command interface, gripper: `effort` + PID

## Contents

- `description/models/panda/panda.urdf` — Panda URDF with `<ros2_control>` block (gz_ros2_control/GazeboSimSystem)
- `config/ros_control.yaml` — JointTrajectoryController config for arm (position) and hand (effort+PID)
- `launch/gazebo.launch.py` — launches gz-sim, spawns robot, starts controllers
- `rviz/rviz.rviz` — RViz2 config

## Launch (via icgnet_main)

`gazebo.launch.py` is included by `icgnet_main/launch/world.launch.py`, which passes the world SDF
and adds the camera bridge and TF. Do not run it directly.

```bash
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch icgnet_main world.launch.py
```

See `TESTING_GUIDE.md` at the workspace root for the full terminal sequence.
