#pragma once
#include <Arduino.h>
#include <stdint.h>

namespace mc {

struct Time {
  static inline uint32_t ms() { return (uint32_t)millis(); }
  static inline uint64_t us() { return (uint64_t)micros(); }
};

class PeriodicTimer {
 public:
  explicit PeriodicTimer(uint32_t period_ms = 1000) : _period(period_ms), _next(0) {}
  void setPeriod(uint32_t period_ms) { _period = period_ms; }
  void reset(uint32_t now_ms) { _next = now_ms + _period; }
  bool due(uint32_t now_ms) {
    if ((int32_t)(now_ms - _next) >= 0) { _next += _period; return true; }
    return false;
  }
 private:
  uint32_t _period;
  uint32_t _next;
};

} // namespace mc
