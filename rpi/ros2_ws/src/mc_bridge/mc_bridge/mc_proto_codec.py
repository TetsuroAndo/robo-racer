import struct
from typing import Optional, Tuple

MAGIC = b"MC"
VER = 1

TYPE_DRIVE = 0x01
TYPE_KILL = 0x02
TYPE_MODE_SET = 0x03
TYPE_PING = 0x04
TYPE_LOG = 0x10
TYPE_STATUS = 0x11
TYPE_HILS_STATE = 0x12
TYPE_ACK = 0x80

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


def decode_packet(enc: bytes) -> Tuple[Optional[Tuple[int, int, int, bytes]], str]:
    if not enc:
        return None, "empty"
    if enc[-1] == 0:
        enc = enc[:-1]
    raw = cobs_decode(enc)
    if len(raw) < 11:
        return None, "cobs"
    hdr = raw[:9]
    magic, ver, ptype, flags, seq, plen = struct.unpack("<2sBBBHH", hdr)
    if magic != MAGIC or ver != VER:
        return None, "header"
    payload = raw[9:-2]
    if len(payload) != plen:
        return None, "length"
    crc_got = struct.unpack("<H", raw[-2:])[0]
    crc_exp = crc16_ccitt(raw[:-2])
    if crc_got != crc_exp:
        return None, "crc"
    return (ptype, flags, seq, payload), ""


def decode_status(payload: bytes):
    if len(payload) != 10:
        return None
    seq_applied, auto_active, faults, speed, steer, age_ms = struct.unpack(
        "<BBHhhH", payload
    )
    return seq_applied, auto_active, faults, speed, steer, age_ms


def decode_drive(payload: bytes):
    if len(payload) != 8:
        return None
    steer, speed, ttl_ms, dist_mm = struct.unpack("<hhHH", payload)
    return steer, speed, ttl_ms, dist_mm


def decode_hils_state(payload: bytes):
    if len(payload) != 9:
        return None
    ts_ms, throttle_raw, steer_cdeg, flags = struct.unpack("<IhhB", payload)
    return ts_ms, throttle_raw, steer_cdeg, flags


def decode_log(payload: bytes):
    if len(payload) < 1:
        return None
    level = payload[0]
    text = payload[1:].decode("utf-8", errors="replace")
    return level, text
