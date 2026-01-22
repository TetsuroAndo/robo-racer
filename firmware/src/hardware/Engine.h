#pragma once
#include <Arduino.h>

class Engine {
public:
	void begin();

	// speed: -255..255 (マイナス=後退, プラス=前進, 0=停止)
	void setSpeed(int speed);

	void stop();

private:
	int _prev_speed;

	void applyPWM(uint8_t rpwm, uint8_t lpwm);
	void outputSpeed(int cur);
};
