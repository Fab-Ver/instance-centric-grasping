#!/usr/bin/env python3
"""
Direct JTC test — bypasses MoveIt2 and pymoveit2 entirely.
Sends a FollowJointTrajectory goal directly to panda_arm_controller.

If this moves the robot: Gazebo controller works, problem is in MoveIt2 layer.
If this also fails: problem is in Gazebo hardware interface or controller config.

Usage:
  ros2 run icgnet_main direct_jtc_test
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

JOINTS = ['panda_joint1', 'panda_joint2', 'panda_joint3',
          'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']

READY = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
REACH = [0.0, -0.3,   0.0, -1.8,   0.0,  1.5,  0.785]


def send_traj(node, client, start, end, duration_sec=3):
    traj = JointTrajectory()
    traj.joint_names = JOINTS

    p0 = JointTrajectoryPoint()
    p0.positions = list(start)
    p0.velocities = [0.0] * 7
    p0.time_from_start = Duration(sec=0, nanosec=0)

    p1 = JointTrajectoryPoint()
    p1.positions = list(end)
    p1.velocities = [0.0] * 7
    p1.time_from_start = Duration(sec=duration_sec, nanosec=0)

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = traj
    goal.trajectory.points = [p0, p1]

    node.get_logger().info(f'Sending trajectory (duration={duration_sec}s)...')
    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future, timeout_sec=5.0)
    goal_handle = send_future.result()

    if goal_handle is None or not goal_handle.accepted:
        node.get_logger().error('Goal rejected or timed out!')
        return False

    node.get_logger().info('Goal accepted. Waiting for result...')
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=duration_sec + 15.0)
    result = result_future.result()

    if result is None:
        node.get_logger().error('Result timed out!')
        return False

    code = result.result.error_code
    msg  = result.result.error_string
    if code == FollowJointTrajectory.Result.SUCCESSFUL:
        node.get_logger().info('SUCCESS')
        return True
    else:
        node.get_logger().error(f'FAILED: error_code={code}, msg="{msg}"')
        return False


def main():
    rclpy.init()
    node = Node('direct_jtc_test')
    client = ActionClient(node, FollowJointTrajectory,
                          '/panda_arm_controller/follow_joint_trajectory')

    node.get_logger().info('Waiting for /panda_arm_controller/follow_joint_trajectory...')
    if not client.wait_for_server(timeout_sec=10.0):
        node.get_logger().error('Action server not available after 10s. Is world.launch.py running?')
        rclpy.shutdown()
        return

    node.get_logger().info('=== Moving READY → REACH ===')
    ok = send_traj(node, client, READY, REACH, duration_sec=3)
    if ok:
        import time; time.sleep(1.0)
        node.get_logger().info('=== Moving REACH → READY ===')
        send_traj(node, client, REACH, READY, duration_sec=3)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
