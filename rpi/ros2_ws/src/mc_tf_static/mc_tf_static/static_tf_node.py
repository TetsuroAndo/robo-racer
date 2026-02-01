import math
import sys

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


class StaticTfNode(Node):
    def __init__(self) -> None:
        super().__init__("mc_tf_static")

        self.declare_parameter("parent_frame", "base_link")
        self.declare_parameter("child_frame", "laser")
        self.declare_parameter("translation_x", 0.0)
        self.declare_parameter("translation_y", 0.0)
        self.declare_parameter("translation_z", 0.0)
        self.declare_parameter("rotation_roll", 0.0)
        self.declare_parameter("rotation_pitch", 0.0)
        self.declare_parameter("rotation_yaw", 0.0)

        self._broadcaster = StaticTransformBroadcaster(self)
        self._publish_transform()

    def _publish_transform(self) -> None:
        parent_frame = str(self.get_parameter("parent_frame").value)
        child_frame = str(self.get_parameter("child_frame").value)
        tx = float(self.get_parameter("translation_x").value)
        ty = float(self.get_parameter("translation_y").value)
        tz = float(self.get_parameter("translation_z").value)
        roll = float(self.get_parameter("rotation_roll").value)
        pitch = float(self.get_parameter("rotation_pitch").value)
        yaw = float(self.get_parameter("rotation_yaw").value)

        if not parent_frame:
            raise ValueError("parent_frame must be non-empty")
        if not child_frame:
            raise ValueError("child_frame must be non-empty")

        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = tx
        transform.transform.translation.y = ty
        transform.transform.translation.z = tz
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        self._broadcaster.sendTransform(transform)
        self.get_logger().info(
            f"publishing static TF {parent_frame} -> {child_frame}"
        )


def main() -> None:
    rclpy.init()
    node: StaticTfNode | None = None
    try:
        node = StaticTfNode()
        rclpy.spin(node)
    except Exception as exc:
        logger = rclpy.logging.get_logger("mc_tf_static")
        logger.error(f"fatal error: {exc}")
        sys.exit(1)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
