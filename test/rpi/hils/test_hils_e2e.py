"""
HILS E2E integration test (seriald + sim_esp32d).

@brief
  Uses PTY relay to connect seriald and sim_esp32d, then drives mc_proto frames
  through seriald IPC and validates STATUS/ACK behavior.
"""

import os
import select
import shutil
import socket
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest


MAGIC = b"MC"
VER = 1

TYPE_DRIVE = 0x01
TYPE_KILL = 0x02
TYPE_MODE_SET = 0x03
TYPE_PING = 0x04
TYPE_STATUS = 0x11
TYPE_ACK = 0x80

FLAG_ACK_REQ = 1 << 0

FAULT_KILL = 1 << 0
FAULT_TTL = 1 << 2
FAULT_AUTO_INACTIVE = 1 << 3


def crc16_ccitt(data: bytes) -> int:
    """@brief CRC16-CCITT-FALSE for mc_proto frames."""
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
    """@brief COBS encode (0x00-free) for framing."""
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
    """@brief COBS decode."""
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
    """@brief Build mc_proto packet (COBS + CRC + 0x00)."""
    hdr = struct.pack(
        "<2sBBBHH", MAGIC, VER, ptype, flags, seq & 0xFFFF, len(payload) & 0xFFFF
    )
    body = hdr + payload
    crc = crc16_ccitt(body)
    decoded = body + struct.pack("<H", crc)
    enc = cobs_encode(decoded) + b"\x00"
    return enc


def decode_packet(enc: bytes):
    """@brief Decode mc_proto packet; returns tuple or None on error."""
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


def wait_for_socket(path: Path, timeout_s: float) -> socket.socket | None:
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


def recv_frame(sock: socket.socket, timeout_s: float) -> bytes:
    end = time.time() + timeout_s
    while time.time() < end:
        r, _, _ = select.select([sock], [], [], 0.05)
        if not r:
            continue
        data = sock.recv(4096)
        if data:
            return data
    return b""


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


@pytest.mark.skipif(
    sys.platform != "linux",
    reason="requires Linux AF_UNIX SOCK_SEQPACKET support",
)
@pytest.mark.skipif(shutil.which("g++") is None, reason="g++ not available")
def test_hils_e2e_seriald_sim_esp32d(tmp_path: Path):
    repo_root = Path(__file__).resolve().parents[3]
    seriald_dir = repo_root / "rpi/apps/seriald"
    sim_dir = repo_root / "rpi/apps/sim_esp32d"
    stub_dir = repo_root / "test/rpi/hils/cpp"
    seriald_bin = tmp_path / "seriald"
    sim_bin = tmp_path / "sim_esp32d"
    stub_bin = tmp_path / "racerd_stub"

    build_seriald = [
        "g++",
        "-std=c++17",
        "-Wall",
        "-Wextra",
        "-O2",
        "-pthread",
        "-I",
        str(seriald_dir / "include"),
        "-I",
        str(seriald_dir / "src"),
        "-I",
        str(repo_root / "shared/proto/include"),
        "-I",
        str(repo_root / "rpi/lib/mc_ipc/include"),
        "-I",
        str(repo_root / "rpi/lib/mc_serial/include"),
        "-I",
        str(repo_root / "rpi/lib/mc_core/include"),
        str(seriald_dir / "src/main.cpp"),
        str(repo_root / "shared/proto/src/mcproto.cpp"),
        str(repo_root / "rpi/lib/mc_ipc/src/UdsSeqPacket.cpp"),
        str(repo_root / "rpi/lib/mc_serial/src/Uart.cpp"),
        str(repo_root / "rpi/lib/mc_core/src/Log.cpp"),
        str(repo_root / "rpi/lib/mc_core/src/Time.cpp"),
        "-o",
        str(seriald_bin),
    ]

    build_sim = [
        "g++",
        "-std=c++17",
        "-Wall",
        "-Wextra",
        "-O2",
        "-pthread",
        "-I",
        str(repo_root / "shared/proto/include"),
        "-I",
        str(repo_root / "rpi/lib/mc_serial/include"),
        "-I",
        str(repo_root / "rpi/lib/mc_core/include"),
        str(sim_dir / "src/main.cpp"),
        str(repo_root / "shared/proto/src/mcproto.cpp"),
        str(repo_root / "rpi/lib/mc_serial/src/Uart.cpp"),
        str(repo_root / "rpi/lib/mc_core/src/Log.cpp"),
        str(repo_root / "rpi/lib/mc_core/src/Time.cpp"),
        "-o",
        str(sim_bin),
    ]

    build_stub = [
        "g++",
        "-std=c++17",
        "-Wall",
        "-Wextra",
        "-O2",
        "-I",
        str(repo_root / "shared/proto/include"),
        str(stub_dir / "racerd_stub.cpp"),
        str(repo_root / "shared/proto/src/mcproto.cpp"),
        "-o",
        str(stub_bin),
    ]

    build = subprocess.run(build_seriald, capture_output=True, text=True)
    assert build.returncode == 0, build.stdout + build.stderr
    build = subprocess.run(build_sim, capture_output=True, text=True)
    assert build.returncode == 0, build.stdout + build.stderr
    build = subprocess.run(build_stub, capture_output=True, text=True)
    assert build.returncode == 0, build.stdout + build.stderr

    m1, s1 = os.openpty()
    m2, s2 = os.openpty()
    s1_name = os.ttyname(s1)
    s2_name = os.ttyname(s2)

    stop = threading.Event()
    relay = threading.Thread(target=relay_loop, args=(stop, m1, m2), daemon=True)
    relay.start()

    sock_path = Path("/tmp/seriald.sock")
    log_path = Path("/tmp/seriald.log")
    for p in (sock_path, log_path):
        try:
            p.unlink()
        except FileNotFoundError:
            pass

    seriald_proc = subprocess.Popen(
        [
            str(seriald_bin),
            "--dev",
            s1_name,
            "--baud",
            "115200",
            "--sock",
            str(sock_path),
            "--log",
            str(log_path),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        cwd=str(seriald_dir),
    )

    sim_proc = subprocess.Popen(
        [
            str(sim_bin),
            "--dev",
            s2_name,
            "--baud",
            "115200",
            "--status-ms",
            "20",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        cwd=str(sim_dir),
    )

    uds = None
    try:
        uds = wait_for_socket(sock_path, 2.5)

        stub_run = subprocess.run(
            [str(stub_bin), str(sock_path)], capture_output=True, text=True
        )
        assert stub_run.returncode == 0, stub_run.stdout + stub_run.stderr

        status = None
        deadline = time.time() + 1.0
        while time.time() < deadline:
            pkt = decode_packet(recv_frame(uds, 0.2))
            if pkt and pkt[0] == TYPE_STATUS:
                status = pkt
                break
        assert status is not None, "no STATUS received"
        _, _, _, payload = status
        seq_applied, auto_active, faults, speed, steer, _age = struct.unpack(
            "<BBHhhH", payload
        )
        assert auto_active == 1
        assert speed == 200
        assert steer == 100
        assert seq_applied == 2
        assert (faults & FAULT_AUTO_INACTIVE) == 0

        # TTL expire
        time.sleep(0.08)
        status = None
        deadline = time.time() + 1.0
        while time.time() < deadline:
            pkt = decode_packet(recv_frame(uds, 0.2))
            if pkt and pkt[0] == TYPE_STATUS:
                status = pkt
                if struct.unpack("<BBHhhH", status[3])[2] & FAULT_TTL:
                    break
        assert status is not None, "no STATUS after TTL"
        _, _, _, payload = status
        _seq_applied, _auto_active, faults, speed, steer, _age = struct.unpack(
            "<BBHhhH", payload
        )
        assert (faults & FAULT_TTL) != 0
        assert speed == 0
        assert steer == 0

        # KILL
        uds.send(build_packet(TYPE_KILL, 3, b"", 0))
        status = None
        deadline = time.time() + 1.0
        while time.time() < deadline:
            pkt = decode_packet(recv_frame(uds, 0.2))
            if pkt and pkt[0] == TYPE_STATUS:
                status = pkt
                if struct.unpack("<BBHhhH", status[3])[2] & FAULT_KILL:
                    break
        assert status is not None, "no STATUS after KILL"
        _, _, _, payload = status
        _seq_applied, _auto_active, faults, speed, steer, _age = struct.unpack(
            "<BBHhhH", payload
        )
        assert (faults & FAULT_KILL) != 0
        assert speed == 0
        assert steer == 0

        # PING -> ACK
        uds.send(build_packet(TYPE_PING, 4, b"", FLAG_ACK_REQ))
        ack = None
        deadline = time.time() + 1.0
        while time.time() < deadline:
            pkt = decode_packet(recv_frame(uds, 0.2))
            if pkt and pkt[0] == TYPE_ACK:
                ack = pkt
                break
        assert ack is not None, "no ACK received"
        assert ack[2] == 4

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
