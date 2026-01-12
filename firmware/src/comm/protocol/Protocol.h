#pragma once
#include <stdint.h>

namespace mc::proto {

// Packet framing is handled by PacketSerial (COBS/SLIP).
// This is the *payload* format after framing.
//
// Layout (little-endian):
//   u8  version
//   u8  msg_type
//   u8  seq
//   u8  flags
//   u16 payload_len
//   u16 crc16_ccitt   (computed over [version..payload bytes], excluding crc field itself)
//   u8  payload[payload_len]
//
// RPi side can implement the same: decode COBS, validate CRC, then parse.

static const uint8_t VERSION = 1;

enum class MsgType : uint8_t {
  Ping        = 0x01,
  Heartbeat   = 0x02,

  // Commands (RPi -> ESP32)
  SetMode     = 0x10, // payload: u8 mode
  SetManual   = 0x11, // payload: i16 throttle_milli (-1000..1000), i16 steer_milli (-1000..1000)
  SetPrefer   = 0x12, // payload: u8 prefer_right (0/1)

  Estop       = 0x1F, // payload: none
  ClearEstop  = 0x20, // payload: none

  // Telemetry (ESP32 -> RPi)
  Status      = 0x80, // payload: see StatusPayload

  // Logging (ESP32 -> RPi)
  Log         = 0x90  // payload: LogPayloadHeader + message bytes
};

enum class Mode : uint8_t {
  Idle = 0,
  Manual = 1,
  Auto = 2
};

#pragma pack(push, 1)

struct Header {
  uint8_t version;
  uint8_t msg_type;
  uint8_t seq;
  uint8_t flags;
  uint16_t payload_len;
  uint16_t crc16;
};

struct StatusPayload {
  uint32_t now_ms;

  uint8_t mode;
  uint8_t estop; // 0/1
  uint8_t prefer_right; // 0/1
  uint8_t reserved0;

  uint16_t dist_mm;  // 0 if invalid
  int16_t yaw_cdeg;  // centi-degrees
  int16_t yawRate_cdeg_s;

  int16_t throttle_cmd_milli; // -1000..1000
  int16_t steer_cmd_milli;    // -1000..1000

  uint16_t loop_hz; // approximate
  uint16_t reserved1;
};

struct LogPayloadHeader {
  uint32_t now_ms;
  uint8_t level;
  uint8_t topic;
  uint16_t msg_len;
};

#pragma pack(pop)

} // namespace mc::proto
