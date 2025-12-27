#pragma once
#include "control/State.h"
#include "hardware/Sensors.h"
#include <common/Math.h>
#include <common/SlewRateLimiter.h>

namespace mc {

class MotionController {
 public:
  MotionController(float yawKp, float throttleMax, float rateUp, float rateDn);

  void reset();

  // Applies heading hold around reference heading if enabled.
  ControlCommand compute(
      const RuntimeState& rs,
      const SensorData& s,
      const ControlCommand& target,
      float dt_s);

 private:
  float _yawKp;
  float _throttleMax;
  mc::SlewRateLimiter _throttleLimiter;

  float _lastDt = 0.01f;
};

} // namespace mc
