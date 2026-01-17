#include "Drive.h"

void Drive::begin() {
  pinMode(PIN_REN, OUTPUT);
  pinMode(PIN_LEN, OUTPUT);

  digitalWrite(PIN_REN, HIGH);
  digitalWrite(PIN_LEN, HIGH);

  ledcSetup(CH_RPWM, PWM_FREQ, PWM_RES);
  ledcSetup(CH_LPWM, PWM_FREQ, PWM_RES);

  ledcAttachPin(PIN_RPWM, CH_RPWM);
  ledcAttachPin(PIN_LPWM, CH_LPWM);

  stop();
}

void Drive::applyPWM(uint8_t rpwm, uint8_t lpwm) {
  ledcWrite(CH_RPWM, rpwm);
  ledcWrite(CH_LPWM, lpwm);
}

void Drive::setSpeed(int speed) {
  speed = constrain(speed, -255, 255);

  // Enable 常時ON（安全のため stop() ではPWM=0にする）
  digitalWrite(PIN_REN, HIGH);
  digitalWrite(PIN_LEN, HIGH);

  if (speed > 0) {
    // 前進: RPWMのみ
    applyPWM((uint8_t)speed, 0);
  } else if (speed < 0) {
    // 後退: LPWMのみ
    applyPWM(0, (uint8_t)(-speed));
  } else {
    stop();
  }
}

void Drive::stop() {
  applyPWM(0, 0);
}
