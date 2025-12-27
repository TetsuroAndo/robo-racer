#pragma once
#include <stdint.h>
#include "comm/protocol/Protocol.h"

namespace mc {

enum class EstopState : uint8_t {
  Clear = 0,
  Active = 1
};

struct ControlCommand {
  float throttle = 0.0f; // [-1..1]
  float steer = 0.0f;    // [-1..1]
};

struct RuntimeState {
  proto::Mode mode = proto::Mode::Auto;
  EstopState estop = EstopState::Clear;
  bool preferRight = true;

  // For heading hold
  bool hasHeadingRef = false;
  float headingRefDeg = 0.0f;

  // For maneuver logic
  uint32_t maneuverStartMs = 0;
  float maneuverYawStartDeg = 0.0f;
};

} // namespace mc
