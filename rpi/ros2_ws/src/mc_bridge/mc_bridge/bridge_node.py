import socket
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import String

from mc_msgs.msg import DriveCmd, HilsState, LogRecord, Status

from .mc_proto_codec import (
    TYPE_DRIVE,
    TYPE_HILS_STATE,
    TYPE_LOG,
    TYPE_STATUS,
    decode_drive,
    decode_hils_state,
    decode_log,
    decode_packet,
    decode_status,
)


class RunIdCache:
    def __init__(self, node: Node) -> None:
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.history = QoSHistoryPolicy.KEEP_LAST

        self._node = node
        self._lock = threading.Lock()
        self._run_id = ""
        self._sub = node.create_subscription(
            String,
            "/mc/run_id",
            self._on_run_id,
            qos,
        )

    @property
    def run_id(self) -> str:
        with self._lock:
            return self._run_id

    def _on_run_id(self, msg: String) -> None:
        if msg.data:
            with self._lock:
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
                self._node.get_logger().warning("run_id not set; dropping /mc/log")
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
        self._decode_errors = 0
        self._crc_errors = 0
        self._last_decode_warn_ns = 0
        self._last_crc_warn_ns = 0

        self.declare_parameter("demo_log", False)
        self.declare_parameter("demo_period_sec", 1.0)
        self.declare_parameter(
            "telemetry_sock", "/tmp/roboracer/seriald.telemetry.sock"
        )
        self.declare_parameter(
            "telemetry_compat_sock", "/run/roboracer/seriald.telemetry.sock"
        )
        self.declare_parameter("telemetry_tcp_host", "")
        self.declare_parameter("telemetry_tcp_port", 0)

        qos_reliable = QoSProfile(depth=10)
        qos_reliable.reliability = QoSReliabilityPolicy.RELIABLE
        qos_reliable.history = QoSHistoryPolicy.KEEP_LAST

        self._status_pub = self.create_publisher(Status, "/mc/status", qos_reliable)
        self._drive_pub = self.create_publisher(DriveCmd, "/mc/drive_cmd", qos_reliable)
        self._hils_pub = self.create_publisher(HilsState, "/mc/hils_state", qos_reliable)

        self._stop_evt = threading.Event()
        self._telemetry_thread = threading.Thread(
            target=self._telemetry_loop, name="mc_bridge_telemetry", daemon=True
        )
        self._telemetry_thread.start()
        if self.get_parameter("demo_log").value:
            period = float(self.get_parameter("demo_period_sec").value)
            if period <= 0.0:
                raise ValueError("demo_period_sec must be > 0 when demo_log is true")
            self.create_timer(period, self._demo_log)

    def _demo_log(self) -> None:
        ts_ms = int(self.get_clock().now().nanoseconds / 1_000_000)
        self._log_pub.publish(1, "demo log", ts_ms)

    def _warn_decode(self, msg: str, crc: bool) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if crc:
            self._crc_errors += 1
            if now_ns - self._last_crc_warn_ns > 5_000_000_000:
                self.get_logger().warning(msg)
                self._last_crc_warn_ns = now_ns
        else:
            self._decode_errors += 1
            if now_ns - self._last_decode_warn_ns > 5_000_000_000:
                self.get_logger().warning(msg)
                self._last_decode_warn_ns = now_ns
        ts_ms = int(now_ns / 1_000_000)
        self._log_pub.publish(2, msg, ts_ms)

    def _connect_telemetry(self) -> tuple[socket.socket | None, bool]:
        tcp_host = str(self.get_parameter("telemetry_tcp_host").value)
        tcp_port = int(self.get_parameter("telemetry_tcp_port").value)
        if tcp_host and tcp_port > 0:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(1.0)
                sock.connect((tcp_host, tcp_port))
                self.get_logger().info(
                    f"connected telemetry tcp: {tcp_host}:{tcp_port}"
                )
                return sock, True
            except OSError:
                try:
                    sock.close()
                except Exception:
                    pass
                return None, False

        primary = str(self.get_parameter("telemetry_sock").value)
        compat = str(self.get_parameter("telemetry_compat_sock").value)
        for path in (primary, compat):
            try:
                sock = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
                sock.settimeout(0.5)
                sock.connect(path)
                self.get_logger().info(f"connected telemetry socket: {path}")
                return sock, False
            except OSError:
                try:
                    sock.close()
                except Exception:
                    pass
        return None, False

    def _telemetry_loop(self) -> None:
        while not self._stop_evt.is_set():
            sock, is_stream = self._connect_telemetry()
            if sock is None:
                time.sleep(0.5)
                continue
            buf = bytearray()
            try:
                while not self._stop_evt.is_set():
                    try:
                        data = sock.recv(4096)
                    except socket.timeout:
                        continue
                    except OSError:
                        break
                    if not data:
                        break
                    frames = []
                    if is_stream:
                        buf.extend(data)
                        while True:
                            try:
                                end = buf.index(0)
                            except ValueError:
                                break
                            frame = bytes(buf[: end + 1])
                            del buf[: end + 1]
                            frames.append(frame)
                    else:
                        frames.append(data)
                        for frame in frames:
                            pkt, err = decode_packet(frame)
                            if pkt is None:
                                self._warn_decode(f"decode_error: {err}", err == "crc")
                                continue
                            ptype, _flags, _seq, payload = pkt
                            now_ns = self.get_clock().now().nanoseconds
                            if ptype == TYPE_STATUS:
                                dec = decode_status(payload)
                            if dec is None:
                                self._warn_decode("decode_error: status_len", False)
                                continue
                            msg = Status()
                            (
                                msg.seq_applied,
                                msg.auto_active,
                                msg.faults,
                                msg.speed_mm_s,
                                msg.steer_cdeg,
                                msg.age_ms,
                            ) = dec
                            self._status_pub.publish(msg)
                        elif ptype == TYPE_DRIVE:
                            dec = decode_drive(payload)
                            if dec is None:
                                self._warn_decode("decode_error: drive_len", False)
                                continue
                            msg = DriveCmd()
                            msg.steer_cdeg, msg.speed_mm_s, msg.ttl_ms, msg.dist_mm = dec
                            self._drive_pub.publish(msg)
                        elif ptype == TYPE_HILS_STATE:
                            dec = decode_hils_state(payload)
                            if dec is None:
                                self._warn_decode("decode_error: hils_len", False)
                                continue
                            msg = HilsState()
                            msg.timestamp, msg.throttle_raw, msg.steer_cdeg, msg.flags = dec
                            self._hils_pub.publish(msg)
                            elif ptype == TYPE_LOG:
                                dec = decode_log(payload)
                                if dec is None:
                                    self._warn_decode("decode_error: log_len", False)
                                    continue
                                level, text = dec
                                ts_ms = int(now_ns / 1_000_000)
                                self._log_pub.publish(level, text, ts_ms)
            finally:
                try:
                    sock.close()
                except Exception:
                    pass

    def shutdown(self) -> None:
        self._stop_evt.set()
        if self._telemetry_thread.is_alive():
            self._telemetry_thread.join(timeout=1.0)


def main() -> int:
    node: BridgeNode | None = None
    try:
        rclpy.init()
        node = BridgeNode()
        rclpy.spin(node)
        return 0
    except Exception as exc:
        logger = rclpy.logging.get_logger("mc_bridge")
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
