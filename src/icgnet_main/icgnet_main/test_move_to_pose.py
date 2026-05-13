#!/usr/bin/env python3
"""
Test MoveIt2 pipeline for Franka Panda.

Phase 1 (joint goal): move between two joint configs — no IK needed.
Phase 2 (pose goal):  move TCP to a Cartesian position — uses IK.
Phase 3 (gripper):    open and close fingers.

Usage:
  ros2 run icgnet_main test_move_to_pose --ros-args -p phase:=1
  ros2 run icgnet_main test_move_to_pose --ros-args -p phase:=2
  ros2 run icgnet_main test_move_to_pose --ros-args -p phase:=3
  ros2 run icgnet_main test_move_to_pose   # all phases
"""

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2.robots import panda as robot

READY = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
REACH = [0.0, -0.3,   0.0, -1.8,   0.0,  1.5,  0.785]


def main():
    rclpy.init()
    node = Node('test_move_to_pose')
    node.declare_parameter('phase', 0)
    phase = node.get_parameter('phase').get_parameter_value().integer_value

    cb = ReentrantCallbackGroup()
    arm = MoveIt2(
        node=node,
        joint_names=robot.joint_names(),
        base_link_name=robot.base_link_name(),
        end_effector_name=robot.end_effector_name(),
        group_name=robot.MOVE_GROUP_ARM,
        callback_group=cb,
    )
    arm.max_velocity = 0.3
    arm.max_acceleration = 0.3
    arm.orientation_tolerance = 0.05

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    node.create_rate(1.0).sleep()

    # ── Phase 1: joint goal ───────────────────────────────────────────────
    if phase in (0, 1):
        node.get_logger().info('Phase 1: moving to REACH joint config')
        arm.move_to_configuration(joint_positions=REACH)
        arm.wait_until_executed()
        node.get_logger().info('Phase 1: REACH done, returning to READY')
        arm.move_to_configuration(joint_positions=READY)
        arm.wait_until_executed()
        node.get_logger().info('Phase 1: complete')

    # ── Phase 2: Cartesian pose goal ──────────────────────────────────────
    if phase in (0, 2):
        node.get_logger().info('Phase 2: moving TCP to [0.4, 0.0, 0.5]')
        arm.move_to_pose(position=[0.4, 0.0, 0.5], quat_xyzw=[0.0, 0.0, 0.0, 1.0])
        arm.wait_until_executed()
        node.get_logger().info('Phase 2: returning to READY')
        arm.move_to_configuration(joint_positions=READY)
        arm.wait_until_executed()
        node.get_logger().info('Phase 2: complete')

    # ── Phase 3: gripper ─────────────────────────────────────────────────
    if phase in (0, 3):
        from pymoveit2 import MoveIt2Gripper
        gripper_cb = ReentrantCallbackGroup()
        gripper = MoveIt2Gripper(
            node=node,
            gripper_joint_names=robot.gripper_joint_names(),
            open_gripper_joint_positions=robot.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=robot.CLOSED_GRIPPER_JOINT_POSITIONS,
            gripper_group_name=robot.MOVE_GROUP_GRIPPER,
            callback_group=gripper_cb,
        )
        node.create_rate(0.5).sleep()
        node.get_logger().info('Phase 3: opening gripper')
        gripper.open()
        gripper.wait_until_executed()
        node.get_logger().info('Phase 3: closing gripper')
        gripper.close()
        gripper.wait_until_executed()
        node.get_logger().info('Phase 3: complete')

    node.get_logger().info('=== Test complete ===')
    rclpy.shutdown()
    executor_thread.join(timeout=3.0)


if __name__ == '__main__':
    main()
