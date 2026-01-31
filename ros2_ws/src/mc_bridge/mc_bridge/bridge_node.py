import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import String

from mc_msgs.msg import LogRecord


class RunIdCache:
    def __init__(self, node: Node) -> None:
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.history = QoSHistoryPolicy.KEEP_LAST

        self._node = node
        self._run_id = ""
        self._sub = node.create_subscription(
            String,
            "/mc/run_id",
            self._on_run_id,
            qos,
        )

    @property
    def run_id(self) -> str:
        return self._run_id

    def _on_run_id(self, msg: String) -> None:
        if msg.data:
            self._run_id = msg.data


class LogPublisher:
    def __init__(self, node: Node, run_id_cache: RunIdCache) -> None:
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos.history = QoSHistoryPolicy.KEEP_LAST

        self._node = node
        self._run_id_cache = run_id_cache
        self._pub = node.create_publisher(LogRecord, "/mc/log", qos)
        self._last_warn_ns = 0

    def publish(self, level: int, text: str, ts_ms: int) -> bool:
        run_id = self._run_id_cache.run_id
        if not run_id:
            now_ns = self._node.get_clock().now().nanoseconds
            if now_ns - self._last_warn_ns > 5_000_000_000:
                self._node.get_logger().warn("run_id not set; dropping /mc/log")
                self._last_warn_ns = now_ns
            return False

        msg = LogRecord()
        msg.ts_ms = ts_ms
        msg.level = level
        msg.text = text
        msg.run_id = run_id
        self._pub.publish(msg)
        return True


class BridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("mc_bridge")
        self._run_id_cache = RunIdCache(self)
        self._log_pub = LogPublisher(self, self._run_id_cache)

        self.declare_parameter("demo_log", False)
        self.declare_parameter("demo_period_sec", 1.0)
        if self.get_parameter("demo_log").value:
            period = float(self.get_parameter("demo_period_sec").value)
            self.create_timer(period, self._demo_log)

    def _demo_log(self) -> None:
        ts_ms = int(self.get_clock().now().nanoseconds / 1_000_000)
        self._log_pub.publish(1, "demo log", ts_ms)


def main() -> None:
    rclpy.init()
    node = BridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
