#pragma once
#include "control/State.h"
#include "hardware/Sensors.h"
#include <common/Time.h>

namespace mc {

class Strategy {
 public:
  Strategy();

  // Main autonomous policy
  ControlCommand updateAuto(RuntimeState& rs, const SensorData& s, uint32_t nowMs);

 private:
  enum class AutoSubstate : uint8_t {
    Cruise = 0,
    Brake,
    Reverse,
    Turn
  };

  AutoSubstate _sub = AutoSubstate::Cruise;
  uint32_t _subStartMs = 0;

  void enter(RuntimeState& rs, AutoSubstate st, uint32_t nowMs, const SensorData& s);

  bool obstacleClose(const SensorData& s, uint16_t thrMm) const;
};

} // namespace mc
