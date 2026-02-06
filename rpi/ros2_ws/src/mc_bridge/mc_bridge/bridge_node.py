import json
import math
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
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

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


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _quat_to_rpy(qx: float, qy: float, qz: float, qw: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.asin(_clamp(sinp, -1.0, 1.0))
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def _forward_vec_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float]:
    _ = roll
    cp = math.cos(pitch)
    return (math.cos(yaw) * cp, math.sin(yaw) * cp, math.sin(pitch))


def _parse_kv_int(text: str, key: str) -> int | None:
    needle = key + "="
    idx = text.find(needle)
    if idx < 0:
        return None
    start = idx + len(needle)
    end = start
    if end < len(text) and text[end] == "-":
        end += 1
    while end < len(text) and text[end].isdigit():
        end += 1
    if end == start or (end == start + 1 and text[start] == "-"):
        return None
    try:
        return int(text[start:end])
    except ValueError:
        return None


class TfCache:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._tf = {}

    def update(self, msg: TFMessage) -> None:
        with self._lock:
            for t in msg.transforms:
                self._tf[(t.header.frame_id, t.child_frame_id)] = t

    def get(self, parent: str, child: str):
        with self._lock:
            return self._tf.get((parent, child))


def _get_trans_rot(obj):
    if hasattr(obj, "transform"):
        t = obj.transform.translation
        r = obj.transform.rotation
    else:
        t = obj.position
        r = obj.orientation
    return t, r


def _compose_tf(a, b):
    at, aq = _get_trans_rot(a)
    bt, bq = _get_trans_rot(b)
    ax = at.x
    ay = at.y
    az = at.z
    bx = bt.x
    by = bt.y
    bz = bt.z
    ar, ap, ayaw = _quat_to_rpy(aq.x, aq.y, aq.z, aq.w)
    # rotate b translation by a rotation (yaw/pitch only)
    cp = math.cos(ap)
    sp = math.sin(ap)
    cy = math.cos(ayaw)
    sy = math.sin(ayaw)
    rx = cy * cp
    ry = sy * cp
    rz = sp
    tx = ax + rx * bx - ry * by
    ty = ay + ry * bx + rx * by
    tz = az + rz * bx + (cp * bz)
    # quaternion multiply a * b
    qx = aq.w * bq.x + aq.x * bq.w + aq.y * bq.z - aq.z * bq.y
    qy = aq.w * bq.y - aq.x * bq.z + aq.y * bq.w + aq.z * bq.x
    qz = aq.w * bq.z + aq.x * bq.y - aq.y * bq.x + aq.z * bq.w
    qw = aq.w * bq.w - aq.x * bq.x - aq.y * bq.y - aq.z * bq.z
    return (tx, ty, tz, qx, qy, qz, qw)


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
        self.declare_parameter("ros_telemetry_hz", 10.0)
        self.declare_parameter("ros_telemetry_log", True)
        self.declare_parameter("ros_telemetry_stdout", True)
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

        self._tf_cache = TfCache()
        self._odom = None
        self._imu = None
        self._tsd20 = None
        self._odom_sub = self.create_subscription(
            Odometry, "/odom", self._on_odom, qos_reliable
        )
        self._imu_sub = self.create_subscription(Imu, "/imu", self._on_imu, qos_reliable)
        tf_qos = QoSProfile(depth=100)
        tf_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        tf_qos.history = QoSHistoryPolicy.KEEP_LAST
        self._tf_sub = self.create_subscription(TFMessage, "/tf", self._on_tf, tf_qos)
        tf_static_qos = QoSProfile(depth=1)
        tf_static_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        tf_static_qos.reliability = QoSReliabilityPolicy.RELIABLE
        tf_static_qos.history = QoSHistoryPolicy.KEEP_LAST
        self._tf_static_sub = self.create_subscription(
            TFMessage, "/tf_static", self._on_tf_static, tf_static_qos
        )

        self._stop_evt = threading.Event()
        self._telemetry_thread = threading.Thread(
            target=self._telemetry_loop, name="mc_bridge_telemetry", daemon=True
        )
        self._telemetry_thread.start()
        hz = float(self.get_parameter("ros_telemetry_hz").value)
        if hz > 0.0:
            period = 1.0 / hz
            self.create_timer(period, self._emit_ros_telemetry)
        if self.get_parameter("demo_log").value:
            period = float(self.get_parameter("demo_period_sec").value)
            if period <= 0.0:
                raise ValueError("demo_period_sec must be > 0 when demo_log is true")
            self.create_timer(period, self._demo_log)

    def _demo_log(self) -> None:
        ts_ms = int(self.get_clock().now().nanoseconds / 1_000_000)
        self._log_pub.publish(1, "demo log", ts_ms)

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _on_imu(self, msg: Imu) -> None:
        self._imu = msg

    def _on_tf(self, msg: TFMessage) -> None:
        self._tf_cache.update(msg)

    def _on_tf_static(self, msg: TFMessage) -> None:
        self._tf_cache.update(msg)

    def _update_tsd20_from_log(self, text: str) -> None:
        if not text.startswith("tsd20:"):
            return
        ready = _parse_kv_int(text, "ready")
        valid = _parse_kv_int(text, "valid")
        mm = _parse_kv_int(text, "mm")
        fails = _parse_kv_int(text, "fails")
        period = _parse_kv_int(text, "period")
        now_ns = self.get_clock().now().nanoseconds
        self._tsd20 = {
            "ready": 1 if ready and ready != 0 else 0,
            "valid": 1 if valid and valid != 0 else 0,
            "mm": mm if mm is not None else 0,
            "fails": fails if fails is not None else 0,
            "period_ms": period if period is not None else 0,
            "ts_ms": int(now_ns / 1_000_000),
        }

    def _emit_ros_telemetry(self) -> None:
        run_id = self._run_id_cache.run_id
        now_ns = self.get_clock().now().nanoseconds
        now_us = int(now_ns / 1_000)
        odom = self._odom
        imu = self._imu

        payload: dict = {"t_us": now_us, "type": "ros2_telemetry", "run_id": run_id}

        if odom is not None:
            pos = odom.pose.pose.position
            ori = odom.pose.pose.orientation
            roll, pitch, yaw = _quat_to_rpy(ori.x, ori.y, ori.z, ori.w)
            fx, fy, fz = _forward_vec_from_rpy(roll, pitch, yaw)
            lin = odom.twist.twist.linear
            ang = odom.twist.twist.angular
            payload["odom"] = {
                "frame_id": odom.header.frame_id,
                "child_frame_id": odom.child_frame_id,
                "pos": {"x": pos.x, "y": pos.y, "z": pos.z},
                "quat": {"x": ori.x, "y": ori.y, "z": ori.z, "w": ori.w},
                "rpy_deg": {
                    "roll": math.degrees(roll),
                    "pitch": math.degrees(pitch),
                    "yaw": math.degrees(yaw),
                },
                "dir": {"x": fx, "y": fy, "z": fz},
                "lin_vel": {"x": lin.x, "y": lin.y, "z": lin.z},
                "ang_vel": {"x": ang.x, "y": ang.y, "z": ang.z},
            }

            tf_map = self._tf_cache.get("map", odom.header.frame_id)
            if tf_map is not None:
                tx, ty, tz, qx, qy, qz, qw = _compose_tf(tf_map, odom.pose.pose)
                r2, p2, y2 = _quat_to_rpy(qx, qy, qz, qw)
                f2x, f2y, f2z = _forward_vec_from_rpy(r2, p2, y2)
                payload["map"] = {
                    "frame_id": "map",
                    "child_frame_id": odom.child_frame_id,
                    "pos": {"x": tx, "y": ty, "z": tz},
                    "quat": {"x": qx, "y": qy, "z": qz, "w": qw},
                    "rpy_deg": {
                        "roll": math.degrees(r2),
                        "pitch": math.degrees(p2),
                        "yaw": math.degrees(y2),
                    },
                    "dir": {"x": f2x, "y": f2y, "z": f2z},
                }

        if imu is not None:
            la = imu.linear_acceleration
            av = imu.angular_velocity
            payload["imu_raw"] = {
                "frame_id": imu.header.frame_id,
                "lin_accel": {"x": la.x, "y": la.y, "z": la.z},
                "ang_vel": {"x": av.x, "y": av.y, "z": av.z},
            }

        if self._tsd20 is not None:
            payload["tsd20"] = self._tsd20

        text = json.dumps(payload, separators=(",", ":"))
        if self.get_parameter("ros_telemetry_stdout").value:
            self.get_logger().info(text)
        if self.get_parameter("ros_telemetry_log").value:
            ts_ms = int(now_ns / 1_000_000)
            self._log_pub.publish(1, text, ts_ms)

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
                            self._update_tsd20_from_log(text)
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
