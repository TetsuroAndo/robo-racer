#include "Engine.h"
#include "../config/Config.h"

void Engine::begin() {
	pinMode(cfg::ENGINE_PIN_REN, OUTPUT);
	pinMode(cfg::ENGINE_PIN_LEN, OUTPUT);
	digitalWrite(cfg::ENGINE_PIN_REN, HIGH);
	digitalWrite(cfg::ENGINE_PIN_LEN, HIGH);

	ledcSetup(cfg::ENGINE_CHANNEL_RPWM, cfg::ENGINE_PWM_FREQ_HZ,
			  cfg::ENGINE_PWM_RES_BITS);
	ledcSetup(cfg::ENGINE_CHANNEL_LPWM, cfg::ENGINE_PWM_FREQ_HZ,
			  cfg::ENGINE_PWM_RES_BITS);
	ledcAttachPin(cfg::ENGINE_PIN_RPWM, cfg::ENGINE_CHANNEL_RPWM);
	ledcAttachPin(cfg::ENGINE_PIN_LPWM, cfg::ENGINE_CHANNEL_LPWM);

	stop();
}

void Engine::applyPWM(uint8_t rpwm, uint8_t lpwm) {
	ledcWrite(cfg::ENGINE_CHANNEL_RPWM, rpwm);
	ledcWrite(cfg::ENGINE_CHANNEL_LPWM, lpwm);
}

void Engine::outputSpeed(int cur) {
	digitalWrite(cfg::ENGINE_PIN_REN, HIGH);
	digitalWrite(cfg::ENGINE_PIN_LEN, HIGH);

	if (cur > 0)
		applyPWM((uint8_t)cur, 0);
	else if (cur < 0)
		applyPWM(0, (uint8_t)(-cur));
	else
		applyPWM(0, 0);
}

void Engine::setTarget(int speed_pwm) {
	_tgt =
		constrain(speed_pwm, -cfg::ENGINE_SPEED_LIMIT, cfg::ENGINE_SPEED_LIMIT);
}

void Engine::setRateLimits(float rate_up, float rate_down) {
	_lim.setRates(rate_up, rate_down);
}

void Engine::control(float dt_s) {
	const uint32_t now_us = micros();
	if (_dead_until_us != 0 && (int32_t)(now_us - _dead_until_us) < 0) {
		if (_cur != 0) {
			_cur = 0;
			outputSpeed(0);
		}
		return;
	}
	if (_dead_until_us != 0)
		_dead_until_us = 0;

	float y = _lim.update((float)_tgt, dt_s);
	int next = (int)lroundf(y);
	next = constrain(next, -cfg::ENGINE_SPEED_LIMIT, cfg::ENGINE_SPEED_LIMIT);

	const int next_dir = (next > 0) ? 1 : (next < 0 ? -1 : 0);
	if (_last_dir != 0 && next_dir != 0 && next_dir != _last_dir) {
		_dead_until_us = now_us + cfg::ENGINE_DEADTIME_US;
		_cur = 0;
		_last_dir = 0;
		outputSpeed(0);
		return;
	}

	if (next != _cur) {
		_cur = next;
		outputSpeed(_cur);
		if (next_dir != 0)
			_last_dir = next_dir;
	}
}

void Engine::stop() {
	_tgt = 0;
	_cur = 0;
	_last_dir = 0;
	_dead_until_us = 0;
	_lim.reset(0.0f);
	applyPWM(0, 0);
}
