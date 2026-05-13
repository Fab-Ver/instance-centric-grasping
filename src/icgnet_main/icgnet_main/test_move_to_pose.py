#!/usr/bin/env python3
"""
Test MoveIt2 pipeline: arm joint goal → arm pose goal → gripper open/close.

Phase 1 (joint goal): move arm between two known configurations.
  No IK needed — verifies planning + trajectory execution end-to-end.

Phase 2 (pose goal): move TCP to a Cartesian position.
  Uses IK (KDL). Run only after Phase 1 succeeds.

Phase 3 (gripper): open and close fingers via MoveIt2.

Usage:
  ros2 run icgnet_main test_move_to_pose               # all phases
  ros2 run icgnet_main test_move_to_pose --ros-args -p phase:=1   # joint only
  ros2 run icgnet_main test_move_to_pose --ros-args -p phase:=2   # pose only
  ros2 run icgnet_main test_move_to_pose --ros-args -p phase:=3   # gripper only
"""

import time
from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2, MoveIt2Gripper
from pymoveit2.robots import panda as robot

# "ready" pose (robot starts here thanks to initial_value in URDF)
READY = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
# "reach forward" — arm extends toward the table
REACH = [0.0, -0.3, 0.0, -1.8, 0.0, 1.5, 0.785]


def spin_safe(executor):
    """Spin executor without dying on rclpy action-client race condition (Humble bug)."""
    while rclpy.ok():
        try:
            executor.spin_once(timeout_sec=0.05)
        except Exception:
            pass


def main():
    rclpy.init()
    node = Node('test_move_to_pose')
    node.declare_parameter('phase', 0)  # 0 = all phases

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
    arm.orientation_tolerance = 0.05  # radians — relaxed for KDL IK

    executor = rclpy.executors.MultiThreadedExecutor(4)
    executor.add_node(node)
    executor_thread = Thread(target=spin_safe, args=(executor,), daemon=True)
    executor_thread.start()
    time.sleep(2.0)  # let move_group settle

    # ── Phase 1: joint goal ────────────────────────────────────────────────
    if phase in (0, 1):
        node.get_logger().info('=== Phase 1: joint goal ===')
        node.get_logger().info('Moving to REACH configuration...')
        arm.move_to_configuration(joint_positions=REACH)
        arm.wait_until_executed()
        node.get_logger().info('REACH done.')
        time.sleep(0.5)
        node.get_logger().info('Moving back to READY configuration...')
        arm.move_to_configuration(joint_positions=READY)
        arm.wait_until_executed()
        node.get_logger().info('READY done.')

    # ── Phase 2: Cartesian pose goal ────────────────────────────────────────
    if phase in (0, 2):
        node.get_logger().info('=== Phase 2: Cartesian pose goal ===')
        # target: 40 cm in front, 50 cm height, gripper pointing forward (identity)
        node.get_logger().info('Moving TCP to [0.4, 0.0, 0.5] identity orientation...')
        arm.move_to_pose(
            position=[0.4, 0.0, 0.5],
            quat_xyzw=[0.0, 0.0, 0.0, 1.0],
        )
        arm.wait_until_executed()
        node.get_logger().info('Pose move done (check logs for planning errors).')
        time.sleep(0.5)
        node.get_logger().info('Returning to READY...')
        arm.move_to_configuration(joint_positions=READY)
        arm.wait_until_executed()

    # ── Phase 3: gripper ───────────────────────────────────────────────────
    if phase in (0, 3):
        node.get_logger().info('=== Phase 3: gripper ===')
        gripper = MoveIt2Gripper(
            node=node,
            gripper_joint_names=robot.gripper_joint_names(),
            open_gripper_joint_positions=robot.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=robot.CLOSED_GRIPPER_JOINT_POSITIONS,
            gripper_group_name=robot.MOVE_GROUP_GRIPPER,
            callback_group=cb,
        )
        time.sleep(0.5)  # let gripper action clients register
        node.get_logger().info('Opening gripper...')
        gripper.open()
        gripper.wait_until_executed()
        node.get_logger().info('Closing gripper...')
        gripper.close()
        gripper.wait_until_executed()
        node.get_logger().info('Gripper done.')

    node.get_logger().info('=== Test complete ===')
    rclpy.shutdown()
    executor_thread.join(timeout=3.0)


if __name__ == '__main__':
    main()
