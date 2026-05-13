#!/usr/bin/env python3
"""
Test: move Panda arm to a target pose, then open/close gripper.
Run after launching world.launch.py (Gazebo + move_group).

Usage:
  ros2 run icgnet_main test_move_to_pose
"""

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2, MoveIt2Gripper
from pymoveit2.robots import panda as robot


def main():
    rclpy.init()

    node = Node('test_move_to_pose')
    node.declare_parameter('position', [0.4, 0.0, 0.5])
    node.declare_parameter('quat_xyzw', [1.0, 0.0, 0.0, 0.0])

    cb = ReentrantCallbackGroup()

    arm = MoveIt2(
        node=node,
        joint_names=robot.joint_names(),
        base_link_name=robot.base_link_name(),
        end_effector_name=robot.end_effector_name(),
        group_name=robot.MOVE_GROUP_ARM,
        callback_group=cb,
    )
    gripper = MoveIt2Gripper(
        node=node,
        gripper_joint_names=robot.gripper_joint_names(),
        open_gripper_joint_positions=robot.OPEN_GRIPPER_JOINT_POSITIONS,
        closed_gripper_joint_positions=robot.CLOSED_GRIPPER_JOINT_POSITIONS,
        gripper_group_name=robot.MOVE_GROUP_GRIPPER,
        callback_group=cb,
    )

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    node.create_rate(1.0).sleep()

    arm.max_velocity = 0.3
    arm.max_acceleration = 0.3

    position = node.get_parameter('position').get_parameter_value().double_array_value
    quat_xyzw = node.get_parameter('quat_xyzw').get_parameter_value().double_array_value

    node.get_logger().info(f'Moving arm to position={list(position)}, quat_xyzw={list(quat_xyzw)}')
    arm.move_to_pose(position=position, quat_xyzw=quat_xyzw)
    arm.wait_until_executed()
    node.get_logger().info('Arm move complete.')

    node.get_logger().info('Closing gripper...')
    gripper.close()
    gripper.wait_until_executed()

    node.get_logger().info('Opening gripper...')
    gripper.open()
    gripper.wait_until_executed()

    node.get_logger().info('Test complete.')
    rclpy.shutdown()
    executor_thread.join()


if __name__ == '__main__':
    main()
