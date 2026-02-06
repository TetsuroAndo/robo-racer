#pragma once
#include <Arduino.h>
#include "../../lib/common/SlewRateLimiter.h"

class Engine {
public:
	void begin();

	void setTarget(int speed_pwm);
	void setRateLimits(float rate_up, float rate_down);
	void stop();

	void control(float dt_s);

	int current() const { return _cur; }
	int target() const { return _tgt; }

private:
	int _cur = 0;
	int _tgt = 0;
	int _last_dir = 0;
	uint32_t _dead_until_us = 0;

	mc::SlewRateLimiter _lim{800.0f, 1200.0f, 0.0f};

	void applyPWM(uint8_t rpwm, uint8_t lpwm);
	void outputSpeed(int cur);
};
