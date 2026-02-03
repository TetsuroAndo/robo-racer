import mmap
import struct
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


SHM_PATH = "/dev/shm/lidar_scan"
POINTS = 181
ANGLE_MIN = -1.5707963267948966
ANGLE_MAX = 1.5707963267948966
ANGLE_INCREMENT = 3.141592653589793 / 180.0
RANGE_MIN = 0.005
RANGE_MAX = 20.0


def _read_scan(mm: mmap.mmap):
    raw = mm.read(4 + POINTS * 4)
    mm.seek(0)
    if len(raw) < 4 + POINTS * 4:
        return None
    seq = struct.unpack_from("<I", raw, 0)[0]
    distances = struct.unpack_from("<" + "i" * POINTS, raw, 4)
    return seq, distances


class LidarBridge(Node):
    def __init__(self) -> None:
        super().__init__("lidar_bridge")
        self._pub = self.create_publisher(LaserScan, "/scan", 10)
        self._last_seq = 0
        self._mm = None
        self._declare_params()
        self._timer = self.create_timer(0.05, self._tick)

    def _declare_params(self) -> None:
        self.declare_parameter("shm_path", SHM_PATH)
        self.declare_parameter("range_min", RANGE_MIN)
        self.declare_parameter("range_max", RANGE_MAX)
        self.declare_parameter("frame_id", "laser")

    def _connect(self) -> bool:
        if self._mm is not None:
            return True
        path = str(self.get_parameter("shm_path").value)
        try:
            with open(path, "rb") as f:
                self._mm = mmap.mmap(f.fileno(), 4 + POINTS * 4, access=mmap.ACCESS_READ)
            self.get_logger().info(f"connected shm {path}")
            return True
        except FileNotFoundError:
            return False
        except OSError as exc:
            self.get_logger().warning(f"failed to open shm {path}: {exc}")
            return False

    def _tick(self) -> None:
        if not self._connect():
            return
        scan = _read_scan(self._mm)
        if scan is None:
            return
        seq, distances = scan
        if seq == 0 or seq == self._last_seq:
            return
        self._last_seq = seq

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("frame_id").value)
        msg.angle_min = ANGLE_MIN
        msg.angle_max = ANGLE_MAX
        msg.angle_increment = ANGLE_INCREMENT
        msg.range_min = float(self.get_parameter("range_min").value)
        msg.range_max = float(self.get_parameter("range_max").value)

        ranges = []
        range_max = msg.range_max
        for dist_mm in distances:
            if dist_mm <= 0:
                ranges.append(float("inf"))
            else:
                ranges.append(min(dist_mm / 1000.0, range_max))
        msg.ranges = ranges
        self._pub.publish(msg)

    def shutdown(self) -> None:
        if self._mm is not None:
            self._mm.close()
            self._mm = None


def main() -> int:
    node: LidarBridge | None = None
    try:
        rclpy.init()
        node = LidarBridge()
        rclpy.spin(node)
        return 0
    except Exception as exc:
        logger = rclpy.logging.get_logger("lidar_bridge")
        logger.error(f"fatal error: {exc}")
        return 1
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
