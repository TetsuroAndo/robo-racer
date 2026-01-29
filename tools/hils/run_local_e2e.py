"""Local HILS E2E runner (seriald + sim_esp32d).

This script starts seriald and sim_esp32d with a PTY relay, sends
MODE_SET/DRIVE via seriald IPC, then prints STATUS frames for a short window.
"""

from __future__ import annotations

import argparse
import os
import select
import socket
import struct
import subprocess
import threading
import time
from pathlib import Path


MAGIC = b"MC"
VER = 1

TYPE_DRIVE = 0x01
TYPE_MODE_SET = 0x03
TYPE_STATUS = 0x11


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


def build_packet(ptype: int, seq: int, payload: bytes) -> bytes:
    hdr = struct.pack(
        "<2sBBBHH", MAGIC, VER, ptype, 0, seq & 0xFFFF, len(payload) & 0xFFFF
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


def relay_loop(stop: threading.Event, m1: int, m2: int) -> None:
    while not stop.is_set():
        r, _, _ = select.select([m1, m2], [], [], 0.05)
        for fd in r:
            try:
                data = os.read(fd, 4096)
            except OSError:
                continue
            if not data:
                continue
            other = m2 if fd == m1 else m1
            try:
                os.write(other, data)
            except OSError:
                continue


def wait_for_socket(path: Path, timeout_s: float) -> socket.socket:
    end = time.time() + timeout_s
    last_err = ""
    while time.time() < end:
        try:
            uds = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
            uds.connect(str(path))
            return uds
        except OSError as e:
            last_err = str(e)
            time.sleep(0.05)
    raise RuntimeError(f"failed to connect socket: {last_err}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seriald", required=True)
    parser.add_argument("--sim", required=True)
    parser.add_argument("--sock", default="/tmp/seriald.sock")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--steer-cdeg", type=int, default=100)
    parser.add_argument("--speed-mm-s", type=int, default=200)
    parser.add_argument("--ttl-ms", type=int, default=200)
    parser.add_argument("--drive-every-ms", type=int, default=50)
    args = parser.parse_args()

    m1, s1 = os.openpty()
    m2, s2 = os.openpty()
    s1_name = os.ttyname(s1)
    s2_name = os.ttyname(s2)

    stop = threading.Event()
    relay = threading.Thread(target=relay_loop, args=(stop, m1, m2), daemon=True)
    relay.start()

    sock_path = Path(args.sock)
    try:
        sock_path.unlink()
    except FileNotFoundError:
        pass

    seriald_proc = subprocess.Popen(
        [
            args.seriald,
            "--dev",
            s1_name,
            "--baud",
            str(args.baud),
            "--sock",
            str(sock_path),
            "--log",
            "/tmp/seriald.log",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )

    sim_proc = subprocess.Popen(
        [
            args.sim,
            "--dev",
            s2_name,
            "--baud",
            str(args.baud),
            "--status-ms",
            "50",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )

    uds = None
    try:
        uds = wait_for_socket(sock_path, 2.5)

        # MODE_SET AUTO
        uds.send(build_packet(TYPE_MODE_SET, 1, bytes([1])))
        drive_seq = 2
        next_drive = 0.0

        end = time.time() + args.duration
        while time.time() < end:
            now = time.time()
            if args.drive_every_ms > 0 and now >= next_drive:
                drive_payload = struct.pack(
                    "<hhHH",
                    args.steer_cdeg,
                    args.speed_mm_s,
                    args.ttl_ms,
                    0,
                )
                uds.send(build_packet(TYPE_DRIVE, drive_seq, drive_payload))
                drive_seq = (drive_seq + 1) & 0xFFFF
                next_drive = now + (args.drive_every_ms / 1000.0)

            r, _, _ = select.select([uds], [], [], 0.2)
            if not r:
                continue
            data = uds.recv(4096)
            pkt = decode_packet(data)
            if not pkt or pkt[0] != TYPE_STATUS:
                continue
            _ptype, _flags, _seq, payload = pkt
            seq_applied, auto_active, faults, speed, steer, age = struct.unpack(
                "<BBHhhH", payload
            )
            print(
                f"STATUS seq={seq_applied} auto={auto_active} speed={speed} "
                f"steer={steer} age={age} faults=0x{faults:04x}"
            )

    finally:
        if uds is not None:
            uds.close()
        for proc in (sim_proc, seriald_proc):
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.communicate(timeout=1)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.communicate(timeout=1)
        stop.set()
        relay.join(timeout=1)
        for fd in (m1, s1, m2, s2):
            try:
                os.close(fd)
            except OSError:
                pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
