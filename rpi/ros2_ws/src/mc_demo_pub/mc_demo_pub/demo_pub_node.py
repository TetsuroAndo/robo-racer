import math
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(value, max_value))


class DemoPubNode(Node):
    def __init__(self) -> None:
        super().__init__("mc_demo_pub")

        self.declare_parameter("topic", "/scan")
        self.declare_parameter("frame_id", "laser")
        self.declare_parameter("publish_rate", 5.0)
        self.declare_parameter("angle_min", -math.pi)
        self.declare_parameter("angle_max", math.pi)
        self.declare_parameter("angle_increment", math.radians(1.0))
        self.declare_parameter("range_min", 0.05)
        self.declare_parameter("range_max", 10.0)
        self.declare_parameter("range_mean", 3.0)
        self.declare_parameter("range_variation", 0.25)

        self._topic = str(self.get_parameter("topic").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._publish_rate = float(self.get_parameter("publish_rate").value)
        self._angle_min = float(self.get_parameter("angle_min").value)
        self._angle_max = float(self.get_parameter("angle_max").value)
        self._angle_increment = float(self.get_parameter("angle_increment").value)
        self._range_min = float(self.get_parameter("range_min").value)
        self._range_max = float(self.get_parameter("range_max").value)
        self._range_mean = float(self.get_parameter("range_mean").value)
        self._range_variation = float(self.get_parameter("range_variation").value)

        if self._publish_rate <= 0.0:
            raise ValueError("publish_rate must be > 0")
        if self._angle_increment <= 0.0:
            raise ValueError("angle_increment must be > 0")
        if self._angle_max <= self._angle_min:
            raise ValueError("angle_max must be > angle_min")
        if self._range_max <= self._range_min:
            raise ValueError("range_max must be > range_min")

        self._count = int(round((self._angle_max - self._angle_min) / self._angle_increment)) + 1
        if self._count <= 0:
            raise ValueError("computed scan count is invalid")

        self._scan_time = 1.0 / self._publish_rate
        self._time_increment = self._scan_time / float(self._count)
        self._phase = 0.0

        self._pub = self.create_publisher(LaserScan, self._topic, 10)
        self.create_timer(self._scan_time, self._publish_scan)
        self.get_logger().info(
            f"publishing demo LaserScan on {self._topic} ({self._count} beams)"
        )

    def _publish_scan(self) -> None:
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.angle_min = self._angle_min
        msg.angle_max = self._angle_max
        msg.angle_increment = self._angle_increment
        msg.time_increment = self._time_increment
        msg.scan_time = self._scan_time
        msg.range_min = self._range_min
        msg.range_max = self._range_max

        phase = self._phase
        self._phase += 0.15

        ranges = []
        for i in range(self._count):
            value = self._range_mean + self._range_variation * math.sin(phase + i * 0.08)
            ranges.append(clamp(value, self._range_min, self._range_max))
        msg.ranges = ranges

        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node: DemoPubNode | None = None
    try:
        node = DemoPubNode()
        rclpy.spin(node)
    except Exception as exc:
        logger = rclpy.logging.get_logger("mc_demo_pub")
        logger.error(f"fatal error: {exc}")
        sys.exit(1)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
