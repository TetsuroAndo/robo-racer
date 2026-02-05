#pragma once
#include <stdint.h>

namespace mc {

// Simple slew-rate limiter for float signals (e.g., throttle).
// rate_up: max increase per second, rate_down: max decrease per second.
class SlewRateLimiter {
 public:
  SlewRateLimiter(float rate_up, float rate_down, float initial = 0.0f)
  : _rateUp(rate_up), _rateDown(rate_down), _y(initial) {}

  void reset(float v) { _y = v; }
  float value() const { return _y; }
  void setRates(float rate_up, float rate_down) {
    _rateUp = rate_up;
    _rateDown = rate_down;
  }

  float update(float target, float dt_s) {
    const float dy = target - _y;
    const float maxUp = _rateUp * dt_s;
    const float maxDn = _rateDown * dt_s;
    if (dy > maxUp) _y += maxUp;
    else if (dy < -maxDn) _y -= maxDn;
    else _y = target;
    return _y;
  }

 private:
  float _rateUp;
  float _rateDown;
  float _y;
};

} // namespace mc
