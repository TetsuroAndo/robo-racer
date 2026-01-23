"""
seriald integration test (UART <-> IPC).

@brief
  Runs seriald against a PTY-backed UART and a UNIX IPC socket (seqpacket on
  Linux, dgram on macOS), then verifies UART->IPC and IPC->UART forwarding using
  mc_proto frames.
"""

import os
import shutil
import select
import socket
import struct
import subprocess
import sys
import time
from pathlib import Path

import pytest


MAGIC = b"MC"
VER = 1

TYPE_MODE_SET = 0x03
TYPE_LOG = 0x10
TYPE_STATUS = 0x11

FLAG_ACK_REQ = 1 << 0


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


def read_frame_from_fd(fd: int, timeout_s: float) -> bytes:
    """@brief Read one COBS frame (0x00-terminated) from fd."""
    buf = bytearray()
    end = time.time() + timeout_s
    while time.time() < end:
        r, _, _ = select.select([fd], [], [], 0.05)
        if not r:
            continue
        chunk = os.read(fd, 4096)
        if not chunk:
            continue
        buf.extend(chunk)
        if 0 in chunk:
            idx = buf.find(0)
            frame = bytes(buf[: idx + 1])
            return frame
    return b""


@pytest.mark.skipif(shutil.which("g++") is None, reason="g++ not available")
def test_seriald_uart_ipc_roundtrip(tmp_path: Path):
    """
    @brief
      IPC -> UART: send MODE_SET and verify UART receives same frame.
      UART -> IPC: send STATUS and verify IPC receives same frame.
      UART -> seriald logger: send LOG and verify log file contains message.
    """
    repo_root = Path(__file__).resolve().parents[3]
    seriald_dir = repo_root / "rpi/apps/seriald"
    bin_path = tmp_path / "seriald"

    build_cmd = [
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
        str(seriald_dir / "src/async_logger.cpp"),
        str(seriald_dir / "src/sink_stdout.cpp"),
        str(seriald_dir / "src/sink_file.cpp"),
        str(repo_root / "shared/proto/src/mcproto.cpp"),
        str(repo_root / "rpi/lib/mc_ipc/src/UdsSeqPacket.cpp"),
        str(repo_root / "rpi/lib/mc_serial/src/Uart.cpp"),
        str(repo_root / "rpi/lib/mc_core/src/Log.cpp"),
        str(repo_root / "rpi/lib/mc_core/src/Time.cpp"),
        "-o",
        str(bin_path),
    ]

    build = subprocess.run(build_cmd, capture_output=True, text=True)
    assert build.returncode == 0, build.stdout + build.stderr

    print("[INFO] built seriald for test:", bin_path)

    master_fd, slave_fd = os.openpty()
    slave_name = os.ttyname(slave_fd)

    sock_path = Path("/tmp/seriald.sock")
    log_path = Path("/tmp/seriald.log")
    try:
        log_path.unlink()
    except FileNotFoundError:
        pass
    try:
        sock_path.unlink()
    except FileNotFoundError:
        pass

    proc = subprocess.Popen(
        [
            str(bin_path),
            "--dev",
            slave_name,
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

    try:
        deadline = time.time() + 2.5
        try:
            sock_path.unlink()
        except FileNotFoundError:
            pass
        uds = None
        use_dgram = sys.platform == "darwin"
        sock_type = socket.SOCK_DGRAM if use_dgram else socket.SOCK_SEQPACKET
        # macOS: AF_UNIX sun_path has a short length limit; tmp_path can exceed it.
        client_path = Path("/tmp") / f"seriald_client_{os.getpid()}.sock"
        last_err = ""
        while time.time() < deadline:
            if proc.poll() is not None:
                break
            try:
                uds = socket.socket(socket.AF_UNIX, sock_type)
                if use_dgram:
                    try:
                        client_path.unlink()
                    except FileNotFoundError:
                        pass
                    uds.bind(str(client_path))
                uds.connect(str(sock_path))
                last_err = ""
                break
            except OSError as e:
                last_err = str(e)
                if uds is not None:
                    uds.close()
                uds = None
                time.sleep(0.05)

        if uds is None:
            proc.terminate()
            try:
                out, err = proc.communicate(timeout=1)
            except subprocess.TimeoutExpired:
                proc.kill()
                out, err = proc.communicate(timeout=1)
            log_text = ""
            if log_path.exists():
                log_text = log_path.read_text(errors="replace")
            assert uds is not None, (
                f"seriald IPC not ready. rc={proc.returncode} last_err={last_err}\n"
                f"stdout:\n{out.decode(errors='replace')}\n"
                f"stderr:\n{err.decode(errors='replace')}\n"
                f"log:\n{log_text}\n"
            )

        # IPC -> UART (MODE_SET)
        mode_payload = bytes([1])
        mode_frame = build_packet(TYPE_MODE_SET, 0x10, mode_payload, FLAG_ACK_REQ)
        print(
            "[INFO] send IPC->UART MODE_SET",
            {
                "type": TYPE_MODE_SET,
                "seq": 0x10,
                "flags": FLAG_ACK_REQ,
                "payload": mode_payload,
            },
        )
        uds.send(mode_frame)

        rx_uart = read_frame_from_fd(master_fd, 1.0)
        print("[INFO] uart rx bytes:", rx_uart)
        assert rx_uart
        dec = decode_packet(rx_uart)
        print("[INFO] uart decoded:", dec)
        assert dec is not None
        ptype, flags, seq, payload = dec
        print(
            "[EXPECT] uart type/flags/seq/payload:",
            TYPE_MODE_SET,
            FLAG_ACK_REQ,
            0x10,
            mode_payload,
        )
        print("[ACTUAL] uart type/flags/seq/payload:", ptype, flags, seq, payload)
        assert ptype == TYPE_MODE_SET
        assert flags == FLAG_ACK_REQ
        assert seq == 0x10
        assert payload == mode_payload

        # UART -> IPC (STATUS)
        status_payload = struct.pack("<BBHhhH", 0x22, 1, 0x0002, -10, 25, 100)
        status_frame = build_packet(TYPE_STATUS, 0x33, status_payload, 0)
        print(
            "[INFO] send UART->IPC STATUS",
            {"type": TYPE_STATUS, "seq": 0x33, "flags": 0, "payload": status_payload},
        )
        os.write(master_fd, status_frame)

        r, _, _ = select.select([uds], [], [], 1.0)
        print("[INFO] ipc recv ready:", bool(r))
        assert r
        rx_ipc = uds.recv(4096)
        print("[INFO] ipc rx bytes:", rx_ipc)
        dec2 = decode_packet(rx_ipc)
        print("[INFO] ipc decoded:", dec2)
        assert dec2 is not None
        ptype2, flags2, seq2, payload2 = dec2
        print(
            "[EXPECT] ipc type/flags/seq/payload:", TYPE_STATUS, 0, 0x33, status_payload
        )
        print("[ACTUAL] ipc type/flags/seq/payload:", ptype2, flags2, seq2, payload2)
        assert ptype2 == TYPE_STATUS
        assert flags2 == 0
        assert seq2 == 0x33
        assert payload2 == status_payload

        # UART -> LOG (LOG payload)
        log_text = "test-log-msg"
        log_payload = bytes([2]) + log_text.encode("utf-8")
        log_frame = build_packet(TYPE_LOG, 0x44, log_payload, 0)
        print(
            "[INFO] send UART->LOG",
            {"type": TYPE_LOG, "seq": 0x44, "flags": 0, "payload": log_payload},
        )
        os.write(master_fd, log_frame)

        # wait log file to contain message
        deadline = time.time() + 2.0
        found = False
        while time.time() < deadline:
            if log_path.exists():
                txt = log_path.read_text(errors="replace")
                if log_text in txt:
                    found = True
                    break
            time.sleep(0.05)
        print("[EXPECT] log contains:", log_text)
        print("[ACTUAL] log found:", found)
        assert found
    finally:
        if "uds" in locals() and uds is not None:
            uds.close()
        if "client_path" in locals() and client_path.exists():
            try:
                client_path.unlink()
            except OSError:
                pass
        proc.terminate()
        try:
            proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            proc.kill()
        os.close(master_fd)
        os.close(slave_fd)
