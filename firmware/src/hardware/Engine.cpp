#include "Engine.h"
#include "../config/Config.h"

void Engine::begin() {
	pinMode(cfg::ENGINE_PIN_REN, OUTPUT);
	pinMode(cfg::ENGINE_PIN_LEN, OUTPUT);

	digitalWrite(cfg::ENGINE_PIN_REN, HIGH);
	digitalWrite(cfg::ENGINE_PIN_LEN, HIGH);

	ledcSetup(cfg::ENGINE_CHANNEL_RPWM, cfg::ENGINE_PWM_FREQ_HZ,
			  cfg::ENGINE_PWM_RESOLUTION_BITS);
	ledcSetup(cfg::ENGINE_CHANNEL_LPWM, cfg::ENGINE_PWM_FREQ_HZ,
			  cfg::ENGINE_PWM_RESOLUTION_BITS);

	ledcAttachPin(cfg::ENGINE_PIN_RPWM, cfg::ENGINE_CHANNEL_RPWM);
	ledcAttachPin(cfg::ENGINE_PIN_LPWM, cfg::ENGINE_CHANNEL_LPWM);

	stop();
}

void Engine::applyPWM(uint8_t rpwm, uint8_t lpwm) {
	ledcWrite(cfg::ENGINE_CHANNEL_RPWM, rpwm);
	ledcWrite(cfg::ENGINE_CHANNEL_LPWM, lpwm);
}

// 1ステップの変化量と、ステップ間隔
static constexpr int SPEED_STEP = cfg::ENGINE_SPEED_STEP; // 0..255スケール
static constexpr int RAMP_DELAY_MS =
	cfg::ENGINE_RAMP_DELAY_MS; // 小さめ推奨（0でも可）

void Engine::outputSpeed(int cur) {
	// Enable 常時ON
	digitalWrite(cfg::ENGINE_PIN_REN, HIGH);
	digitalWrite(cfg::ENGINE_PIN_LEN, HIGH);

	if (cur > 0) {
		applyPWM((uint8_t)cur, 0);
	} else if (cur < 0) {
		applyPWM(0, (uint8_t)(-cur));
	} else {
		applyPWM(0, 0);
	}
}

void Engine::setSpeed(int speed) {
	int target =
		constrain(speed, -cfg::ENGINE_SPEED_LIMIT, cfg::ENGINE_SPEED_LIMIT);
	int cur = _prev_speed;

	// 方向反転は一旦0まで落としてから（ギア/モータ保護＆挙動安定）
	if ((cur > 0 && target < 0) || (cur < 0 && target > 0)) {
		while (cur != 0) {
			cur += (cur > 0) ? -SPEED_STEP : +SPEED_STEP;
			if ((cur > 0 && cur < SPEED_STEP) || (cur < 0 && -cur < SPEED_STEP))
				cur = 0;

			_prev_speed = cur;
			outputSpeed(cur);
			if (RAMP_DELAY_MS)
				delay(RAMP_DELAY_MS);
			else
				delay(0); // yield
		}
	}

	// 目標までランプ
	while (cur != target) {
		int diff = target - cur;
		if (diff > SPEED_STEP)
			diff = SPEED_STEP;
		else if (diff < -SPEED_STEP)
			diff = -SPEED_STEP;

		cur += diff;
		_prev_speed = cur;

		outputSpeed(cur);
		if (RAMP_DELAY_MS)
			delay(RAMP_DELAY_MS);
		else
			delay(0); // yield（WDT回避）
	}
}

void Engine::stop() {
	_prev_speed = 0;
	applyPWM(0, 0);
}
