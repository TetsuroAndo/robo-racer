#!/usr/bin/env python3
"""Steer angle limit test helper (seriald IPC).

This script sweeps steering angles while listening to STATUS frames.
Use manual stop when you hear abnormal sounds; the stop angle is printed.
"""

from __future__ import annotations

import argparse
import os
import select
import socket
import stat
import struct
import subprocess
import sys
import termios
import time
import tty
from contextlib import contextmanager
from dataclasses import dataclass

MAGIC = b"MC"
VER = 1

TYPE_DRIVE = 0x01
TYPE_KILL = 0x02
TYPE_MODE = 0x03
TYPE_STATUS = 0x11


def try_reset_esp32() -> bool:
    if os.environ.get("STEER_TEST_SKIP_RESET") == "1":
        return False
    pulse_reset = None
    try:
        from rpi.reset.esp32_reset import pulse_reset  # type: ignore
    except ModuleNotFoundError:
        try:
            from reset.esp32_reset import pulse_reset  # type: ignore
        except ModuleNotFoundError:
            pulse_reset = None

    if not os.path.exists("/dev/gpiochip0"):
        print("ESP32 reset skipped: /dev/gpiochip0 not found.")
        return False

    try:
        if pulse_reset is not None:
            pulse_reset()
        else:
            script_candidates = [
                os.path.join("playground", "reset_signal", "rpi_reset_esp32.py"),
                os.path.join("tools", "rpi_reset_esp32.py"),
            ]
            script = next(
                (p for p in script_candidates if os.path.exists(p)),
                None,
            )
            if not script:
                print("ESP32 reset skipped: reset module not found.")
                return False
            res = subprocess.run(
                [sys.executable, script],
                check=False,
                capture_output=True,
                text=True,
            )
            if res.returncode != 0:
                msg = res.stderr.strip() or res.stdout.strip() or "unknown error"
                print(f"ESP32 reset skipped: {msg}")
                return False
    except Exception as exc:
        print(f"ESP32 reset skipped: {exc}")
        return False

    time.sleep(0.4)
    print("ESP32 reset pulsed.")
    return True


def default_sock() -> str:
    runtime_dir = os.environ.get("XDG_RUNTIME_DIR")
    candidates = []
    if runtime_dir:
        candidates.append(os.path.join(runtime_dir, "roboracer", "seriald.sock"))
    candidates.extend(
        [
            "/run/roboracer/seriald.sock",
            "/tmp/roboracer/seriald.sock",
            "/tmp/seriald.sock",
        ]
    )
    for path in candidates:
        if os.path.exists(path):
            return path
    return "/tmp/roboracer/seriald.sock"


def check_tmp_socket_dir_safe(sock_path: str) -> None:
    if not sock_path.startswith("/tmp/"):
        return
    sock_dir = os.path.dirname(sock_path)
    try:
        st = os.stat(sock_dir)
    except FileNotFoundError as exc:
        raise SystemExit(f"socket dir missing: {sock_dir}") from exc
    if not stat.S_ISDIR(st.st_mode):
        raise SystemExit(f"socket dir is not a directory: {sock_dir}")
    uid = os.getuid()
    if st.st_uid not in (0, uid):
        raise SystemExit(f"socket dir owner mismatch: {sock_dir}")
    if st.st_mode & (stat.S_IWGRP | stat.S_IWOTH):
        raise SystemExit(f"socket dir is group/world-writable: {sock_dir}")


def crc16_ccitt(data: bytes) -> int:
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
    hdr = struct.pack(
        "<2sBBBHH", MAGIC, VER, ptype, flags, seq & 0xFFFF, len(payload) & 0xFFFF
    )
    body = hdr + payload
    crc = crc16_ccitt(body)
    decoded = body + struct.pack("<H", crc)
    enc = cobs_encode(decoded) + b"\x00"
    return enc


def decode_packet(enc: bytes):
    if not enc:
        return None
    if enc[-1] == 0:
        enc = enc[:-1]
    raw = cobs_decode(enc)
    if len(raw) < 9 + 2:
        return None
    hdr = raw[:9]
    magic, ver, ptype, flags, seq, plen = struct.unpack("<2sBBBHH", hdr)
    if magic != MAGIC or ver != VER:
        return None
    payload = raw[9:-2]
    if len(payload) != plen:
        return None
    crc_got = struct.unpack("<H", raw[-2:])[0]
    crc_exp = crc16_ccitt(raw[:-2])
    if crc_got != crc_exp:
        return None
    return (ptype, flags, seq, payload)


@dataclass
class StatusFrame:
    seq_applied: int
    auto_active: int
    faults: int
    speed_mm_s: int
    steer_cdeg: int
    age_ms: int


def decode_status(payload: bytes) -> StatusFrame | None:
    if len(payload) != 10:
        return None
    seq_applied, auto_active, faults, speed, steer, age = struct.unpack(
        "<BBHhhH", payload
    )
    return StatusFrame(seq_applied, auto_active, faults, speed, steer, age)


@contextmanager
def raw_terminal():
    if not sys.stdin.isatty():
        yield
        return
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        yield
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def build_angles(max_cdeg: int, step_cdeg: int, direction: str) -> list[int]:
    if step_cdeg <= 0:
        raise SystemExit("step-cdeg must be > 0")
    if max_cdeg < 0:
        max_cdeg = -max_cdeg
    angles: list[int] = []
    if direction == "positive":
        angles.extend(range(0, max_cdeg + step_cdeg, step_cdeg))
    elif direction == "negative":
        angles.extend(range(0, -max_cdeg - step_cdeg, -step_cdeg))
    elif direction == "both":
        angles.extend(range(0, max_cdeg + step_cdeg, step_cdeg))
        angles.extend(range(-step_cdeg, -max_cdeg - step_cdeg, -step_cdeg))
    else:
        raise SystemExit(f"unknown direction: {direction}")
    return angles


def send_mode(sock: socket.socket, seq: int, auto: bool) -> None:
    mode = 1 if auto else 0
    payload = bytes([mode])
    sock.send(build_packet(TYPE_MODE, seq, payload))


def send_drive(
    sock: socket.socket,
    seq: int,
    steer_cdeg: int,
    speed_mm_s: int,
    ttl_ms: int,
) -> None:
    payload = struct.pack("<hhHH", steer_cdeg, speed_mm_s, ttl_ms, 0)
    sock.send(build_packet(TYPE_DRIVE, seq, payload))


def send_kill(sock: socket.socket, seq: int) -> None:
    sock.send(build_packet(TYPE_KILL, seq, b"\x00\x00"))


def format_cdeg(cdeg: int) -> str:
    return f"{cdeg / 100.0:.2f}deg ({cdeg} cdeg)"


def ensure_auto(sock: socket.socket, seq: int, timeout_s: float = 2.0) -> tuple[int, StatusFrame | None]:
    end = time.time() + timeout_s
    next_send = 0.0
    last_status: StatusFrame | None = None
    while time.time() < end:
        now = time.time()
        if now >= next_send:
            send_mode(sock, seq, auto=True)
            seq = (seq + 1) & 0xFFFF
            next_send = now + 0.2
        r, _, _ = select.select([sock], [], [], 0.1)
        if not r:
            continue
        data = sock.recv(4096)
        dec = decode_packet(data)
        if not dec or dec[0] != TYPE_STATUS:
            continue
        status = decode_status(dec[3])
        if status is None:
            continue
        last_status = status
        if status.auto_active == 1:
            return seq, last_status
    return seq, last_status


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sock", default=default_sock())
    ap.add_argument("--max-deg", type=float, default=30.0)
    ap.add_argument("--step-cdeg", type=int, default=100)
    ap.add_argument("--dwell-ms", type=int, default=600)
    ap.add_argument("--drive-every-ms", type=int, default=50)
    ap.add_argument("--ttl-ms", type=int, default=200)
    ap.add_argument("--direction", choices=["positive", "negative", "both"], default="both")
    ap.add_argument("--side", choices=["left", "right", "both"], default=None)
    ap.add_argument("--alternate", action="store_true")
    ap.add_argument("--alt-pos-deg", type=float, default=None)
    ap.add_argument("--alt-neg-deg", type=float, default=None)
    ap.add_argument("--alternate-count", type=int, default=0)
    ap.add_argument("--status-every-ms", type=int, default=200)
    ap.add_argument("--speed-mm-s", type=int, default=0)
    args = ap.parse_args()

    if abs(args.max_deg) > 30.0:
        raise SystemExit("max-deg must be within ±30.0 for safety")
    max_cdeg = int(round(args.max_deg * 100.0))
    if max_cdeg == 0:
        raise SystemExit("max-deg must be > 0")

    check_tmp_socket_dir_safe(args.sock)
    try_reset_esp32()

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    sock.connect(args.sock)

    seq = 1
    last_status: StatusFrame | None = None
    last_target_cdeg = 0
    last_print = 0.0

    seq, last_status = ensure_auto(sock, seq)
    if last_status is not None and last_status.auto_active != 1:
        print("AUTO mode is not active. Run 'mode --mode auto' or check other processes.")
        send_mode(sock, seq, auto=False)
        return 1

    if args.alternate:
        pos_deg = args.alt_pos_deg if args.alt_pos_deg is not None else args.max_deg
        neg_deg = args.alt_neg_deg if args.alt_neg_deg is not None else -args.max_deg
        pos_cdeg = int(round(pos_deg * 100.0))
        neg_cdeg = int(round(neg_deg * 100.0))
        if pos_cdeg <= 0:
            raise SystemExit("alt-pos-deg must be > 0 for alternate mode")
        if neg_cdeg >= 0:
            raise SystemExit("alt-neg-deg must be < 0 for alternate mode")
        if abs(pos_cdeg) > max_cdeg or abs(neg_cdeg) > max_cdeg:
            raise SystemExit("alternate angles must be within ±max-deg")

        def angle_iter():
            if args.alternate_count <= 0:
                while True:
                    yield pos_cdeg
                    yield neg_cdeg
            else:
                for _ in range(args.alternate_count):
                    yield pos_cdeg
                    yield neg_cdeg

        angles = angle_iter()
        direction = "alternate"
    else:
        if args.side is not None:
            # left=positive, right=negative (per MANUAL dpad mapping in firmware)
            side_map = {"left": "positive", "right": "negative", "both": "both"}
            direction = side_map[args.side]
        else:
            direction = args.direction
        angles = build_angles(max_cdeg, args.step_cdeg, direction)
    if not angles:
        raise SystemExit("no angles to test")

    print("Steer limit test started.")
    print("Manual stop: press 's' (safe stop), 'k' (kill). Quit: press 'q'.")
    if direction == "alternate":
        print(
            "Alternate: "
            f"+{format_cdeg(pos_cdeg)} / {format_cdeg(neg_cdeg)}, "
            f"dwell={args.dwell_ms}ms, count={args.alternate_count or 'infinite'}"
        )
    else:
        print(
            f"Sweep: max={format_cdeg(max_cdeg)}, step={format_cdeg(args.step_cdeg)}, "
            f"dwell={args.dwell_ms}ms, direction={direction}"
        )

    stop_reason = ""
    stop_action = "safe"
    stop_angle_cdeg = 0

    status_every_s = max(args.status_every_ms, 0) / 1000.0
    with raw_terminal():
        try:
            for target_cdeg in angles:
                last_target_cdeg = target_cdeg
                next_drive = 0.0
                end_time = time.time() + (args.dwell_ms / 1000.0)
                while time.time() < end_time:
                    now = time.time()
                    if now >= next_drive:
                        send_drive(
                            sock,
                            seq,
                            target_cdeg,
                            args.speed_mm_s,
                            args.ttl_ms,
                        )
                        seq = (seq + 1) & 0xFFFF
                        next_drive = now + (args.drive_every_ms / 1000.0)

                    rlist = [sock]
                    if sys.stdin.isatty():
                        rlist.append(sys.stdin)
                    r, _, _ = select.select(rlist, [], [], 0.05)

                    for ready in r:
                        if ready is sock:
                            data = sock.recv(4096)
                            dec = decode_packet(data)
                            if not dec or dec[0] != TYPE_STATUS:
                                continue
                            status = decode_status(dec[3])
                            if status is not None:
                                last_status = status
                        else:
                            key = sys.stdin.read(1)
                            if key in ("s", "q", "k"):
                                if key == "k":
                                    stop_reason = "manual_kill"
                                    stop_action = "kill"
                                elif key == "s":
                                    stop_reason = "manual_stop"
                                    stop_action = "safe"
                                else:
                                    stop_reason = "manual_quit"
                                    stop_action = "safe"
                                stop_angle_cdeg = target_cdeg
                                raise KeyboardInterrupt
                    if (
                        last_status is not None
                        and status_every_s > 0
                        and (now - last_print) >= status_every_s
                    ):
                        last_print = now
                        print(f"cdeg: {last_status.steer_cdeg}")
        except KeyboardInterrupt:
            pass
        finally:
            if stop_action == "kill":
                send_kill(sock, seq)
                seq = (seq + 1) & 0xFFFF
            else:
                send_drive(sock, seq, 0, 0, min(args.ttl_ms, 200))
                seq = (seq + 1) & 0xFFFF
            send_mode(sock, seq, auto=False)

    if not stop_reason:
        stop_reason = "sweep_complete"
        stop_angle_cdeg = last_target_cdeg

    print("")
    print(f"Stop reason: {stop_reason}")
    print(f"Target angle at stop: {format_cdeg(stop_angle_cdeg)}")
    if last_status is not None:
        print(f"Last STATUS steer: {format_cdeg(last_status.steer_cdeg)}")
    else:
        print("Last STATUS steer: (no status received)")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
