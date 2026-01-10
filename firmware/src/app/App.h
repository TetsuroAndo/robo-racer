#pragma once
#include <Arduino.h>
#include <Stream.h>
#include "config/Config.h"
#include "hardware/MotorDriver.h"
#include "hardware/ServoSteering.h"
#include "comm/BleUart.h"

namespace mc {

class App {
 public:
  App();

  void begin();
  void loop();

 private:
  struct RcCommand {
    float throttle = 0.0f;
    float steer = 0.0f;
  };

  class LineParser {
   public:
    bool poll(Stream &stream, RcCommand &out);

   private:
    static constexpr size_t kBufSize = 64;
    char _buf[kBufSize]{};
    size_t _len = 0;

    bool parseLine(const char *line, RcCommand &out);
  };

  MotorDriver _motor{cfg::PIN_RPWM, cfg::PIN_LPWM, cfg::PIN_REN, cfg::PIN_LEN};
  ServoSteering _servo{cfg::PIN_SERVO};
  BleUart _ble{};

  LineParser _serialParser{};
  LineParser _bleParser{};

  RcCommand _cmd{};
  uint32_t _lastCmdMs = 0;

  bool readInputs(uint32_t nowMs);
  void applyOutputs(const RcCommand &cmd);
};

} // namespace mc
