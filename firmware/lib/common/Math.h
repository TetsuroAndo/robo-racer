#pragma once
#include <math.h>

namespace mc {

template <typename T>
static inline T clamp(T v, T lo, T hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

static inline float deg2rad(float d) { return d * (float)M_PI / 180.0f; }
static inline float rad2deg(float r) { return r * 180.0f / (float)M_PI; }

// Wrap angle to [-180, 180)
static inline float wrapDeg180(float deg) {
  while (deg >= 180.0f) deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  return deg;
}

} // namespace mc
