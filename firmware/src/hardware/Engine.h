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
	// ESP32 pins -> IBT-2
	static const int PIN_RPWM = 25; // ESP32 GPIO25 -> IBT2_RPWM
	static const int PIN_LPWM = 26; // ESP32 GPIO26 -> IBT2_LPWM
	static const int PIN_REN = 27;	// ESP32 GPIO27 -> IBT2_R_EN
	static const int PIN_LEN = 14;	// ESP32 GPIO14 -> IBT2_L_EN

	// LEDC PWM
	static const int CH_RPWM = 0;
	static const int CH_LPWM = 1;
	static const int PWM_FREQ = 20000; // 20kHz (聞こえにくい帯域)
	static const int PWM_RES = 8;	   // 8bit -> 0..255

	void applyPWM(uint8_t rpwm, uint8_t lpwm);
	void outputSpeed(int cur);
};
