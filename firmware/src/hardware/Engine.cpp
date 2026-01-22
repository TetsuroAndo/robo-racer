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

void Engine::outputSpeed(int cur) {
	digitalWrite(PIN_REN, HIGH);
	digitalWrite(PIN_LEN, HIGH);

	if (cur > 0)
		applyPWM((uint8_t)cur, 0);
	else if (cur < 0)
		applyPWM(0, (uint8_t)(-cur));
	else
		applyPWM(0, 0);
}

void Engine::setTarget(int speed_pwm) {
	_tgt = constrain(speed_pwm, -255, 255);
}

void Engine::control(float dt_s) {
	float y = _lim.update((float)_tgt, dt_s);
	int next = (int)lroundf(y);
	next = constrain(next, -255, 255);
	if (next != _cur) {
		_cur = next;
		outputSpeed(_cur);
	}
}

void Engine::stop() {
	_tgt = 0;
	_cur = 0;
	_lim.reset(0.0f);
	applyPWM(0, 0);
}
