#pragma once
#include <Arduino.h>
#include "../../lib/common/SlewRateLimiter.h"

class Engine {
public:
	void begin();

	void setTarget(int speed_pwm);
	void stop();

	void control(float dt_s);

	int current() const { return _cur; }
	int target() const { return _tgt; }

private:
	int _cur = 0;
	int _tgt = 0;

	mc::SlewRateLimiter _lim{800.0f, 1200.0f, 0.0f};

	static const int PIN_RPWM = 25;
	static const int PIN_LPWM = 26;
	static const int PIN_REN  = 27;
	static const int PIN_LEN  = 14;
	static const int CH_RPWM  = 0;
	static const int CH_LPWM  = 1;
	static const int PWM_FREQ = 20000;
	static const int PWM_RES  = 8;

	void applyPWM(uint8_t rpwm, uint8_t lpwm);
	void outputSpeed(int cur);
};
