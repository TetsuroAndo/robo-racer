#include "Engine.h"

void Engine::begin() {
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

void Engine::applyPWM(uint8_t rpwm, uint8_t lpwm) {
	ledcWrite(CH_RPWM, rpwm);
	ledcWrite(CH_LPWM, lpwm);
}

// 1ステップの変化量と、ステップ間隔
static constexpr int SPEED_STEP = 4;      // 0..255スケール
static constexpr int RAMP_DELAY_MS = 10;  // 小さめ推奨（0でも可）

void Engine::outputSpeed(int cur) {
  // Enable 常時ON
  digitalWrite(PIN_REN, HIGH);
  digitalWrite(PIN_LEN, HIGH);

  if (cur > 0) {
    applyPWM((uint8_t)cur, 0);
  } else if (cur < 0) {
    applyPWM(0, (uint8_t)(-cur));
  } else {
    applyPWM(0, 0);
  }
}

void Engine::setSpeed(int speed) {
  int target = constrain(speed, -255, 255);
  int cur = _prev_speed;

  // 方向反転は一旦0まで落としてから（ギア/モータ保護＆挙動安定）
  if ((cur > 0 && target < 0) || (cur < 0 && target > 0)) {
    while (cur != 0) {
      cur += (cur > 0) ? -SPEED_STEP : +SPEED_STEP;
      if ((cur > 0 && cur < SPEED_STEP) || (cur < 0 && -cur < SPEED_STEP)) cur = 0;

      _prev_speed = cur;
      outputSpeed(cur);
      if (RAMP_DELAY_MS) delay(RAMP_DELAY_MS);
      else delay(0); // yield
    }
  }

  // 目標までランプ
  while (cur != target) {
    int diff = target - cur;
    if (diff > SPEED_STEP) diff = SPEED_STEP;
    else if (diff < -SPEED_STEP) diff = -SPEED_STEP;

    cur += diff;
    _prev_speed = cur;

    outputSpeed(cur);
    if (RAMP_DELAY_MS) delay(RAMP_DELAY_MS);
    else delay(0); // yield（WDT回避）
  }
}


void Engine::stop() {
	_prev_speed = 0;
	applyPWM(0, 0);
}
