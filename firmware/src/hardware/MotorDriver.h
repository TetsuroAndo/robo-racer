#pragma once
#include <Arduino.h>
#include <common/Math.h>
#include <common/SlewRateLimiter.h>

namespace mc {

class MotorDriver {
 public:
  MotorDriver(int pin_rpwm, int pin_lpwm, int pin_ren, int pin_len);

  void begin(uint32_t pwmFreqHz = 20000, uint8_t pwmResolutionBits = 10);

  // throttle: [-1..1], positive forward
  void setThrottle(float throttle);
  void stop();

  float lastCommand() const { return _lastCmd; }

 private:
  int _pinRPWM, _pinLPWM, _pinREN, _pinLEN;

  uint8_t _chR = 0;
  uint8_t _chL = 1;

  uint16_t _pwmMax = 1023;
  float _lastCmd = 0.0f;
};

} // namespace mc
