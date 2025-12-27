#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <common/Result.h>

namespace mc {

class Tsd20 {
 public:
  explicit Tsd20(uint8_t addr7bit);

  mc::Result begin(TwoWire& wire = Wire);

  // Returns distance in mm. If invalid, returns error.
  mc::Result readDistanceMm(uint16_t& outMm);

  mc::Result setLaserEnabled(bool on);

 private:
  uint8_t _addr;
  TwoWire* _wire = nullptr;

  static const uint8_t REG_DIST_H = 0x00;
  static const uint8_t REG_DIST_L = 0x01;
  static const uint8_t REG_LASER  = 0x02;
};

} // namespace mc
