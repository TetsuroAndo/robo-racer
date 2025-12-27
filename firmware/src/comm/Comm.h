#pragma once
#include <Arduino.h>
#include <stdint.h>
#include "comm/protocol/Protocol.h"
#include <common/Result.h>

namespace mc {

struct HostCommand {
  bool hasMode = false;
  proto::Mode mode = proto::Mode::Idle;

  bool hasManual = false;
  float throttle = 0.0f; // [-1..1]
  float steer = 0.0f;    // [-1..1]

  bool hasPrefer = false;
  bool preferRight = true;

  bool estop = false;
  bool clearEstop = false;

  bool heartbeat = false;
};

// Serial protocol handler (ESP32 <-> RPi).
// Uses PacketSerial (COBS) + CRC16 for robustness.
// NOTE: RPi side is not implemented in this repo; only the interface is defined.
class Comm {
 public:
  Comm();
  mc::Result begin(HardwareSerial& serial);

  // Call frequently from loop()
  void poll();

  // Retrieve and clear the latest command flags.
  HostCommand consumeCommand();

  // Mark heartbeat observed at higher layer (if you want), and also sent.
  uint32_t lastHostSeenMs() const { return _lastHostSeenMs; }

  // Telemetry
  void sendStatus(const proto::StatusPayload& st);

 private:
  void onPacket(const uint8_t* buf, size_t len);

  HostCommand _pending;
  uint32_t _lastHostSeenMs = 0;

  uint8_t _rxSeq = 0;
  uint8_t _txSeq = 0;

  HardwareSerial* _ser = nullptr;

  // Implementation detail: we wrap with PacketSerial if enabled.
  void* _packetSerial = nullptr;
};

} // namespace mc
