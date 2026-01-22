#pragma once
#include <Arduino.h>
#include "../config/Config.h"
#include "../control/ControllerInput.h"

class Logger {
public:
  void begin(uint32_t baud = cfg::LOG_BAUD);

  // 毎ループ1行表示
  void printLine(
    bool connected,
    bool running,
    const PadState& st,
    int speed,
    const char* steer
  );

private:
  bool started_ = false;
};
