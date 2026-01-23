#pragma once
#include <Arduino.h>

class Steer {
public:
  void begin();
  void center();
  void left();
  void right();

  // angle: 0中央, 正: 右, 負: 左
  void setAngle(float angle);

private:
  void writePulseUs_(int us);
};
