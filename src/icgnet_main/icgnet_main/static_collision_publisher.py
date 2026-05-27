import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from shape_msgs.msg import SolidPrimitive


class StaticCollisionPublisher(Node):
    """Publish static collision objects (table) to the MoveIt2 planning scene once at startup."""

    def __init__(self):
        super().__init__('static_collision_publisher')

        self.declare_parameter('table_x', 0.65)
        self.declare_parameter('table_y', 0.0)
        self.declare_parameter('table_z_center', 0.025)
        self.declare_parameter('table_size_x', 0.8)
        self.declare_parameter('table_size_y', 0.8)
        self.declare_parameter('table_size_z', 0.05)
        self.declare_parameter('startup_delay', 5.0)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._pub = self.create_publisher(CollisionObject, '/collision_object', qos)

        delay = self.get_parameter('startup_delay').get_parameter_value().double_value
        self._timer = self.create_timer(delay, self._publish_once)

    def _publish_once(self):
        co = CollisionObject()
        co.header.frame_id = 'world'
        co.header.stamp = self.get_clock().now().to_msg()
        co.id = 'grasp_table'

        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [
            self.get_parameter('table_size_x').get_parameter_value().double_value,
            self.get_parameter('table_size_y').get_parameter_value().double_value,
            self.get_parameter('table_size_z').get_parameter_value().double_value,
        ]
        co.primitives = [prim]

        pose = Pose()
        pose.position.x = self.get_parameter('table_x').get_parameter_value().double_value
        pose.position.y = self.get_parameter('table_y').get_parameter_value().double_value
        pose.position.z = self.get_parameter('table_z_center').get_parameter_value().double_value
        pose.orientation.w = 1.0
        co.primitive_poses = [pose]
        co.operation = CollisionObject.ADD

        self._pub.publish(co)
        self.get_logger().info("Published 'grasp_table' CollisionObject to MoveIt2 planning scene.")
        self._timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = StaticCollisionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
