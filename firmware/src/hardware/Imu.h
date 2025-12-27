#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <common/Result.h>

namespace mc {

// IMU interface (MPU-6050).
// We rely on MPU6050_light to reduce bring-up cost.
// It provides raw gyro values and integrated angles.
// Yaw drift exists (no magnetometer), but it is sufficient for short maneuvers and heading hold.
class Imu {
 public:
  mc::Result begin(TwoWire& wire = Wire);

  // Call frequently; returns true if updated
  bool update();

  // Outputs (degrees, degrees/second)
  float yawDeg() const { return _yawDeg; }
  float yawRateDegS() const { return _yawRateDegS; }

  bool healthy() const { return _healthy; }

 private:
  bool _healthy = false;

  uint32_t _lastMs = 0;
  float _yawDeg = 0.0f;
  float _yawRateDegS = 0.0f;

  // Gyro bias estimation (simple)
  float _biasZ = 0.0f;
  uint32_t _biasSamples = 0;
  bool _biasLocked = false;

  void updateBias(float gyroZ, float dt_s, bool stationary);

  // External lib object (opaque here, defined in cpp to avoid header leakage)
  void* _mpu = nullptr;
};

} // namespace mc
