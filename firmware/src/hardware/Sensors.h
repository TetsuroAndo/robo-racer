#pragma once
#include <stdint.h>

namespace mc {

struct SensorData {
  bool distValid = false;
  uint16_t distMm = 0;

  bool imuValid = false;
  float yawDeg = 0.0f;
  float yawRateDegS = 0.0f;
};

} // namespace mc
