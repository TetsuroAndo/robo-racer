#!/usr/bin/env python3
import argparse
import csv
import os
import select
import socket
import struct
import time
from dataclasses import dataclass


MAGIC = b"MC"
VER = 1

TYPE_DRIVE = 0x01
TYPE_KILL = 0x02
TYPE_MODE_SET = 0x03
TYPE_PING = 0x04
TYPE_STATUS = 0x11
TYPE_IMU_STATUS = 0x13
TYPE_TSD20_STATUS = 0x14
TYPE_ACK = 0x80

FLAG_ACK_REQ = 1 << 0


def crc16_ccitt(data: bytes) -> int:
    """CRC16-CCITT-FALSE for mc_proto frames."""
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8) & 0xFFFF
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def cobs_encode(inp: bytes) -> bytes:
    """COBS encode (0x00-free) for framing."""
    out = bytearray()
    out.append(0)
    code = 1
    code_idx = 0
    for b in inp:
        if b == 0:
            out[code_idx] = code
            code = 1
            code_idx = len(out)
            out.append(0)
        else:
            out.append(b)
            code += 1
            if code == 0xFF:
                out[code_idx] = code
                code = 1
                code_idx = len(out)
                out.append(0)
    out[code_idx] = code
    return bytes(out)


def cobs_decode(inp: bytes) -> bytes:
    """COBS decode."""
    if not inp:
        return b""
    out = bytearray()
    idx = 0
    while idx < len(inp):
        code = inp[idx]
        if code == 0:
            return b""
        idx += 1
        for _ in range(code - 1):
            if idx >= len(inp):
                return b""
            out.append(inp[idx])
            idx += 1
        if code != 0xFF and idx < len(inp):
            out.append(0)
    return bytes(out)


def build_packet(ptype: int, seq: int, payload: bytes, flags: int = 0) -> bytes:
    """Build mc_proto packet (COBS + CRC + 0x00)."""
    hdr = struct.pack(
        "<2sBBBHH", MAGIC, VER, ptype, flags, seq & 0xFFFF, len(payload) & 0xFFFF
    )
    body = hdr + payload
    crc = crc16_ccitt(body)
    decoded = body + struct.pack("<H", crc)
    return cobs_encode(decoded) + b"\x00"


def decode_packet(enc: bytes):
    """Decode mc_proto packet; returns tuple or None on error."""
    if not enc:
        return None
    if enc[-1] == 0:
        enc = enc[:-1]
    raw = cobs_decode(enc)
    if len(raw) < 9 + 2:
        return None
    magic, ver, ptype, flags, seq, plen = struct.unpack("<2sBBBHH", raw[:9])
    if magic != MAGIC or ver != VER:
        return None
    payload = raw[9:-2]
    if len(payload) != plen:
        return None
    crc_got = struct.unpack("<H", raw[-2:])[0]
    if crc_got != crc16_ccitt(raw[:-2]):
        return None
    return (ptype, flags, seq, payload)


@dataclass
class ImuSnap:
    v_est: float = 0.0
    a_long: float = 0.0
    calibrated: int = 0
    brake_mode: int = 0
    age_ms: int = 0


@dataclass
class TsdSnap:
    mm: int = 0
    valid: int = 0
    age_ms: int = 0


def now_ms() -> int:
    return int(time.time() * 1000)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sock", default="/tmp/roboracer/seriald.sock")
    ap.add_argument("--telemetry", default="/tmp/roboracer/seriald.telemetry.sock")
    ap.add_argument("--outdir", default="playground/decel_lab/out")
    ap.add_argument("--method", choices=["coast", "profile"], default="coast")
    ap.add_argument("--speed", type=int, default=2500, help="mm/s")
    ap.add_argument(
        "--trigger-mm",
        type=int,
        default=2000,
        help="start braking when tsd <= this",
    )
    ap.add_argument(
        "--stop-v",
        type=int,
        default=150,
        help="stop when |v_est| <= this",
    )
    ap.add_argument("--ttl", type=int, default=120, help="DRIVE ttl_ms")
    ap.add_argument("--drive-hz", type=float, default=50.0)
    ap.add_argument("--ping-hz", type=float, default=20.0)
    ap.add_argument("--max-sec", type=float, default=8.0)
    args = ap.parse_args()

    if args.drive_hz <= 0:
        ap.error("--drive-hz must be greater than 0")
    if args.ping_hz <= 0:
        ap.error("--ping-hz must be greater than 0")

    os.makedirs(args.outdir, exist_ok=True)

    ctl = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    ctl.connect(args.sock)

    tel = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    tel.connect(args.telemetry)
    tel.setblocking(False)

    seq = 1

    def send(ptype: int, payload: bytes = b"", flags: int = 0) -> None:
        nonlocal seq
        ctl.send(build_packet(ptype, seq, payload, flags))
        seq = (seq + 1) & 0xFFFF

    # AUTO mode
    send(TYPE_MODE_SET, bytes([1]), FLAG_ACK_REQ)

    imu = ImuSnap()
    tsd = TsdSnap()

    def profile_speed(tsd_mm: int) -> int:
        # ざっくり比較用(本番は firmware の Tsd20Limiter が主役)
        # STOP_DISTANCE=500, SLOWDOWN=1000 想定の線形プロファイル
        stop = 500
        slow = 1000
        if tsd_mm <= stop:
            return 0
        if tsd_mm >= slow:
            return args.speed
        r = (tsd_mm - stop) / max(1.0, (slow - stop))
        return int(args.speed * r)

    samples: list[dict[str, int]] = []
    t0 = time.time()
    brake_started = False
    brake_t0 = 0.0
    min_tsd = 10**9

    drive_period = 1.0 / args.drive_hz
    ping_period = 1.0 / args.ping_hz
    last_drive = 0.0
    last_ping = 0.0

    while True:
        t = time.time()
        if t - t0 > args.max_sec:
            break

        # telemetry drain
        while True:
            try:
                data = tel.recv(4096)
            except BlockingIOError:
                break
            dec = decode_packet(data)
            if not dec:
                continue
            ptype, _flags, _rxseq, payload = dec
            if ptype == TYPE_IMU_STATUS and len(payload) == 12:
                # ImuStatusPayload (see shared/proto/include/mc_proto.h)
                # <hhHhHBB:
                #   a_long_mm_s2_le, v_est_mm_s_le, a_brake_cap_mm_s2_le,
                #   yaw_rate_dps_x10_le, age_ms_le, flags, reserved
                a_long, v_est, _a_cap, _yaw_x10, age_ms, flg, _rsv = struct.unpack(
                    "<hhHhHBB", payload
                )
                imu.v_est = float(v_est)
                imu.a_long = float(a_long)
                imu.age_ms = int(age_ms)
                imu.calibrated = 1 if (flg & (1 << 1)) else 0
                imu.brake_mode = 1 if (flg & (1 << 2)) else 0
            elif ptype == TYPE_TSD20_STATUS and len(payload) == 8:
                # Tsd20StatusPayload: <HHHBB
                # mm, period, age_ms, fails, flg
                mm, _period, age_ms, _fails, flg = struct.unpack("<HHHBB", payload)
                tsd.mm = int(mm)
                tsd.age_ms = int(age_ms)
                tsd.valid = 1 if (flg & 0x02) else 0

        if tsd.valid:
            min_tsd = min(min_tsd, tsd.mm)

        # state
        if (not brake_started) and tsd.valid and (tsd.mm <= args.trigger_mm):
            brake_started = True
            brake_t0 = t

        # command
        cmd_speed = args.speed
        if brake_started:
            if args.method == "coast":
                cmd_speed = 0
            elif args.method == "profile":
                cmd_speed = profile_speed(tsd.mm if tsd.valid else 9999)

        # send periodic
        if t - last_ping >= ping_period:
            send(TYPE_PING, b"", 0)  # seriald は常に ACK を返す
            last_ping = t

        if t - last_drive >= drive_period:
            # DrivePayload: <hhHH (steer_cdeg, speed_mm_s, ttl_ms, dist_mm)
            payload = struct.pack("<hhHH", 0, int(cmd_speed), int(args.ttl), 0)
            send(TYPE_DRIVE, payload, 0)
            last_drive = t

        samples.append(
            {
                "t_ms": int((t - t0) * 1000),
                "cmd_speed_mm_s": int(cmd_speed),
                "tsd_valid": int(tsd.valid),
                "tsd_mm": int(tsd.mm),
                "imu_calib": int(imu.calibrated),
                "imu_brake_mode": int(imu.brake_mode),
                "v_est_mm_s": int(imu.v_est),
                "a_long_mm_s2": int(imu.a_long),
                "imu_age_ms": int(imu.age_ms),
                "tsd_age_ms": int(tsd.age_ms),
            }
        )

        # stop condition
        if (
            brake_started
            and imu.age_ms > 0
            and abs(imu.v_est) <= args.stop_v
            and (t - brake_t0) > 0.2
        ):
            break

        time.sleep(0.001)

    # stop
    payload = struct.pack("<hhHH", 0, 0, int(args.ttl), 0)
    send(TYPE_DRIVE, payload, 0)

    ts = time.strftime("%Y%m%d_%H%M%S")
    out = os.path.join(
        args.outdir, f"{ts}_method={args.method}_speed={args.speed}.csv"
    )
    if samples:
        with open(out, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(samples[0].keys()))
            w.writeheader()
            w.writerows(samples)

    t_stop = None
    if brake_started:
        t_stop = int((time.time() - brake_t0) * 1000)

    print(f"[OK] wrote {out}")
    print(
        f"min_tsd_mm={min_tsd if min_tsd != 10**9 else 'n/a'} "
        f"brake_started={brake_started} est_stop_ms={t_stop}"
    )


if __name__ == "__main__":
    main()

