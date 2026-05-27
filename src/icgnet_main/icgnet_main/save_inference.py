import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.serialization import serialize_message

from moveit_msgs.msg import CollisionObject
from icgnet_msgs.msg import GraspArray


class SaveInferenceNode(Node):
    def __init__(self):
        super().__init__('save_inference_node')

        self.declare_parameter('output_dir', os.path.expanduser('~/icgnet_inference_data'))
        self.declare_parameter('collect_window', 2.0)
        self.declare_parameter('object_name', 'target_obj')
        self.declare_parameter('object_sdf_path', '')
        self.declare_parameter('object_x', 0.65)
        self.declare_parameter('object_y', 0.0)
        self.declare_parameter('object_z', 0.05)

        self._output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self._collect_window = self.get_parameter('collect_window').get_parameter_value().double_value
        self._object_name = self.get_parameter('object_name').get_parameter_value().string_value
        self._object_sdf_path = self.get_parameter('object_sdf_path').get_parameter_value().string_value
        self._object_x = self.get_parameter('object_x').get_parameter_value().double_value
        self._object_y = self.get_parameter('object_y').get_parameter_value().double_value
        self._object_z = self.get_parameter('object_z').get_parameter_value().double_value

        self._grasps: GraspArray | None = None
        self._collision_objects: dict[str, CollisionObject] = {}
        self._received_grasps_at: float | None = None
        self._saved = False

        qos_co = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        self.create_subscription(GraspArray, '/icgnet/grasps_rich', self._grasps_cb, 10)
        self.create_subscription(CollisionObject, '/collision_object', self._co_cb, qos_co)
        self.create_timer(0.5, self._check_and_save)

        os.makedirs(self._output_dir, exist_ok=True)
        self.get_logger().info(
            f'SaveInferenceNode ready. Output: {self._output_dir}\n'
            'Trigger ICGNet inference, data will be saved automatically.'
        )

    def _grasps_cb(self, msg: GraspArray):
        if self._grasps is None or self._saved:
            self._grasps = msg
            self._collision_objects.clear()
            self._received_grasps_at = time.time()
            self._saved = False
            self.get_logger().info(f'Received {len(msg.grasps)} grasps. Collecting collision objects...')

    def _co_cb(self, msg: CollisionObject):
        if msg.operation == CollisionObject.ADD:
            self._collision_objects[msg.id] = msg
            self.get_logger().info(f'  CollisionObject: {msg.id} (total: {len(self._collision_objects)})')

    def _check_and_save(self):
        if self._saved or self._grasps is None or self._received_grasps_at is None:
            return
        if time.time() - self._received_grasps_at < self._collect_window:
            return
        self._do_save()
        self._saved = True

    def _do_save(self):
        path = os.path.join(self._output_dir, 'grasps.bin')
        with open(path, 'wb') as f:
            f.write(serialize_message(self._grasps))

        co_ids = []
        for idx, (co_id, co_msg) in enumerate(self._collision_objects.items()):
            path = os.path.join(self._output_dir, f'collision_{idx}.bin')
            with open(path, 'wb') as f:
                f.write(serialize_message(co_msg))
            co_ids.append(co_id)

        meta = {
            'n_grasps': len(self._grasps.grasps),
            'n_collision_objects': len(self._collision_objects),
            'collision_object_ids': co_ids,
            'object_name': self._object_name,
            'object_sdf_path': self._object_sdf_path,
            'object_pose': {'x': self._object_x, 'y': self._object_y, 'z': self._object_z},
        }
        path = os.path.join(self._output_dir, 'meta.json')
        with open(path, 'w') as f:
            json.dump(meta, f, indent=2)

        self.get_logger().info(
            f'[SAVED] {len(self._grasps.grasps)} grasps + '
            f'{len(self._collision_objects)} collision objects → {self._output_dir}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = SaveInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
