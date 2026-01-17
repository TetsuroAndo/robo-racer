#pragma once
#include <Arduino.h>

class Steer {
public:
  void begin();
  void center();
  void left();
  void right();

  // angle: 0..180
  void setAngle(int angle);

private:
  static const int PIN_SERVO = 12; // ESP32 GPIO12 -> DS3218 SIG

  // LEDC for servo (50Hz)
  static const int CH_SERVO = 2;
  static const int SERVO_FREQ = 50;   // 50Hz
  static const int SERVO_RES  = 16;   // 0..65535

  // DS3218想定の一般的なレンジ（必要なら調整）
  static const int PULSE_MIN_US = 500;
  static const int PULSE_MAX_US = 2500;

  // 好みで調整（機体のリンク機構で左右逆なら入れ替えて）
  static const int ANGLE_CENTER = 90;
  static const int ANGLE_LEFT   = 120;
  static const int ANGLE_RIGHT  = 60;

  void writePulseUs_(int us);
};
