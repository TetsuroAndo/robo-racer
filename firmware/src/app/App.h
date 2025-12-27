#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <common/Time.h>
#include "config/Config.h"
#include "hardware/Tsd20.h"
#include "hardware/Imu.h"
#include "hardware/MotorDriver.h"
#include "hardware/ServoSteering.h"
#include "hardware/Sensors.h"
#include "comm/Comm.h"
#include "control/State.h"
#include "control/MotionController.h"
#include "control/Strategy.h"

namespace mc {

class App {
 public:
  App();

  void begin();
  void loop();

 private:
  // Timing
  uint32_t _lastLoopMs = 0;
  uint16_t _loopHz = 0;
  uint32_t _loopCounter = 0;
  uint32_t _loopCounterTs = 0;

  // Core state
  RuntimeState _rs{};
  SensorData _sensors{};

  // Subsystems
  Tsd20 _tsd{cfg::TSD20_I2C_ADDR_7BIT};
  Imu _imu{};
  MotorDriver _motor{cfg::PIN_RPWM, cfg::PIN_LPWM, cfg::PIN_REN, cfg::PIN_LEN};
  ServoSteering _servo{cfg::PIN_SERVO};

  Comm _comm{};
  MotionController _motion{cfg::YAW_KP, cfg::THROTTLE_MAX, cfg::THROTTLE_SLEW_UP, cfg::THROTTLE_SLEW_DN};
  Strategy _strategy{};

  // Outputs (pre-slew)
  ControlCommand _target{};
  ControlCommand _out{};

  mc::PeriodicTimer _telemetryTimer{50}; // 20Hz

  void updateSensors();
  void applyHostCommands(uint32_t nowMs);
  void runStateMachine(uint32_t nowMs, float dt_s);
  void writeActuators();
  void sendTelemetry(uint32_t nowMs);
};

} // namespace mc
