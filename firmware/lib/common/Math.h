#pragma once
#include <cmath>

namespace mc {

template <typename T>
static inline T clamp(T v, T lo, T hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

static constexpr float kPi = 3.14159265358979323846f;

static inline float deg2rad(float d) { return d * kPi / 180.0f; }
static inline float rad2deg(float r) { return r * 180.0f / kPi; }

// Wrap angle to [-180, 180)
static inline float wrapDeg180(float deg) {
  while (deg >= 180.0f) deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  return deg;
}

} // namespace mc
