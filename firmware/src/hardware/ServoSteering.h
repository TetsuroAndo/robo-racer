#pragma once
#include <Arduino.h>
#include <common/Math.h>

namespace mc {

// Servo output using ESP32 LEDC at 50Hz.
class ServoSteering {
 public:
  explicit ServoSteering(int pin);

  // min/max pulse for your servo (DS3218 typical: 500-2500us, but start conservative)
  void begin(uint16_t minUs = 1000, uint16_t maxUs = 2000, uint16_t centerUs = 1500);

  // steer: [-1..1]
  void setNormalized(float steer);
  void center();

  float lastCommand() const { return _last; }

 private:
  int _pin;
  uint8_t _ch = 2;

  uint16_t _minUs = 1000;
  uint16_t _maxUs = 2000;
  uint16_t _centerUs = 1500;

  uint32_t _freq = 50;
  uint8_t _resBits = 16;
  uint32_t _periodUs = 20000;
  uint32_t _maxDuty = 65535;

  float _last = 0.0f;

  void writePulseUs(uint16_t pulseUs);
};

} // namespace mc
