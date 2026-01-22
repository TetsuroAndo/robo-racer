#!/usr/bin/env python3
import argparse
import socket
import struct

MAGIC = b"MC"
VER = 1

TYPE_DRIVE = 0x01
TYPE_KILL  = 0x02
TYPE_MODE  = 0x03

FLAG_ACK_REQ = 1 << 0

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

def build_packet(ptype: int, seq: int, payload: bytes, flags: int = 0) -> bytes:
    hdr = struct.pack("<2sBBBHH", MAGIC, VER, ptype, flags, seq & 0xFFFF, len(payload) & 0xFFFF)
    body = hdr + payload
    crc = crc16_ccitt(body)
    decoded = body + struct.pack("<H", crc)
    enc = cobs_encode(decoded) + b"\x00"
    return enc

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sock", default="/run/roboracer/seriald.sock")
    sub = ap.add_subparsers(dest="cmd", required=True)

    s1 = sub.add_parser("drive")
    s1.add_argument("--steer-cdeg", type=int, required=True)
    s1.add_argument("--speed-mm-s", type=int, required=True)
    s1.add_argument("--ttl-ms", type=int, default=100)
    s1.add_argument("--dist-mm", type=int, default=0)
    s1.add_argument("--seq", type=int, default=1)

    s2 = sub.add_parser("kill")
    s2.add_argument("--seq", type=int, default=1)

    s3 = sub.add_parser("mode")
    s3.add_argument("--mode", choices=["manual","auto"], required=True)
    s3.add_argument("--seq", type=int, default=1)

    args = ap.parse_args()

    if args.cmd == "drive":
        payload = struct.pack("<hhHH", args.steer_cdeg, args.speed_mm_s, args.ttl_ms, args.dist_mm)
        pkt = build_packet(TYPE_DRIVE, args.seq, payload, 0)
    elif args.cmd == "kill":
        payload = b"\x00\x00"
        pkt = build_packet(TYPE_KILL, args.seq, payload, 0)
    else:
        mode = 0 if args.mode == "manual" else 1
        payload = bytes([mode])
        pkt = build_packet(TYPE_MODE, args.seq, payload, 0)

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    sock.sendto(pkt, args.sock)

if __name__ == "__main__":
    main()
