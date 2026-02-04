#!/usr/bin/env python3
import argparse
import os
import select
import socket
import struct
import stat
import time

MAGIC = b"MC"
VER = 1

TYPE_DRIVE = 0x01
TYPE_KILL = 0x02
TYPE_MODE = 0x03
TYPE_PING = 0x04
TYPE_LOG = 0x10
TYPE_STATUS = 0x11
TYPE_ACK = 0x80

FLAG_ACK_REQ = 1 << 0


def default_sock() -> str:
    runtime_dir = os.environ.get("XDG_RUNTIME_DIR")
    if runtime_dir:
        return os.path.join(runtime_dir, "roboracer", "seriald.sock")
    return "/tmp/roboracer/seriald.sock"  # noqa: S108 - dev fallback


def check_tmp_socket_dir_safe(sock_path: str) -> None:
    if not sock_path.startswith("/tmp/"):
        return
    sock_dir = os.path.dirname(sock_path)
    try:
        st = os.stat(sock_dir)
    except FileNotFoundError:
        raise SystemExit(f"socket dir missing: {sock_dir}")
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


def wait_for_ack(sock: socket.socket, seq: int, timeout_ms: int) -> bool:
    deadline = time.time() + (timeout_ms / 1000.0)
    while True:
        remaining = deadline - time.time()
        if remaining <= 0:
            return False
        r, _, _ = select.select([sock], [], [], remaining)
        if not r:
            return False
        data = sock.recv(4096)
        dec = decode_packet(data)
        if not dec:
            continue
        ptype, _, rseq, payload = dec
        if ptype == TYPE_ACK and rseq == (seq & 0xFFFF) and len(payload) == 0:
            return True


def print_frame(dec):
    ptype, flags, seq, payload = dec
    if ptype == TYPE_STATUS and len(payload) == 10:
        seq_applied, auto_active, faults, speed, steer, age_ms = struct.unpack(
            "<BBHhhH", payload
        )
        print(
            f"STATUS seq={seq_applied} auto={auto_active} speed_mm_s={speed} steer_cdeg={steer} age_ms={age_ms} faults=0x{faults:04x}"
        )
    elif ptype == TYPE_LOG and len(payload) >= 1:
        lv = payload[0]
        msg = payload[1:].decode("utf-8", errors="replace")
        print(f"LOG lv={lv} msg={msg}")
    elif ptype == TYPE_ACK and len(payload) == 0:
        print(f"ACK seq={seq}")
    else:
        print(f"RX type=0x{ptype:02x} flags=0x{flags:02x} seq={seq} len={len(payload)}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sock", default=default_sock())
    sub = ap.add_subparsers(dest="cmd", required=True)

    s1 = sub.add_parser("drive")
    s1.add_argument("--steer-cdeg", type=int, required=True)
    s1.add_argument("--speed-mm-s", type=int, required=True)
    s1.add_argument("--ttl-ms", type=int, default=100)
    s1.add_argument("--dist-mm", type=int, default=0)
    s1.add_argument("--seq", type=int, default=1)

    s2 = sub.add_parser("kill")
    s2.add_argument("--seq", type=int, default=1)
    s2.add_argument("--wait-ack", action="store_true")
    s2.add_argument("--timeout-ms", type=int, default=200)

    s3 = sub.add_parser("mode")
    s3.add_argument("--mode", choices=["manual", "auto"], required=True)
    s3.add_argument("--seq", type=int, default=1)
    s3.add_argument("--wait-ack", action="store_true")
    s3.add_argument("--timeout-ms", type=int, default=200)

    s4 = sub.add_parser("listen")
    s4.add_argument("--timeout-ms", type=int, default=0)

    args = ap.parse_args()

    check_tmp_socket_dir_safe(args.sock)

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    sock.connect(args.sock)

    if args.cmd == "listen":
        timeout = None if args.timeout_ms == 0 else args.timeout_ms / 1000.0
        start = time.time()
        while True:
            if timeout is not None and (time.time() - start) > timeout:
                break
            r, _, _ = select.select([sock], [], [], 0.5)
            if not r:
                continue
            data = sock.recv(4096)
            dec = decode_packet(data)
            if dec:
                print_frame(dec)
        return

    if args.cmd == "drive":
        payload = struct.pack(
            "<hhHH", args.steer_cdeg, args.speed_mm_s, args.ttl_ms, args.dist_mm
        )
        pkt = build_packet(TYPE_DRIVE, args.seq, payload, 0)
        sock.send(pkt)
    elif args.cmd == "kill":
        payload = b"\x00\x00"
        flags = FLAG_ACK_REQ if args.wait_ack else 0
        pkt = build_packet(TYPE_KILL, args.seq, payload, flags)
        sock.send(pkt)
        if args.wait_ack:
            ok = wait_for_ack(sock, args.seq, args.timeout_ms)
            print("ACK" if ok else "ACK TIMEOUT")
    else:
        mode = 0 if args.mode == "manual" else 1
        payload = bytes([mode])
        flags = FLAG_ACK_REQ if args.wait_ack else 0
        pkt = build_packet(TYPE_MODE, args.seq, payload, flags)
        sock.send(pkt)
        if args.wait_ack:
            ok = wait_for_ack(sock, args.seq, args.timeout_ms)
            print("ACK" if ok else "ACK TIMEOUT")


if __name__ == "__main__":
    main()
