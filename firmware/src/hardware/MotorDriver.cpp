#include "hardware/MotorDriver.h"

namespace mc {

MotorDriver::MotorDriver(int pin_rpwm, int pin_lpwm, int pin_ren, int pin_len)
	: _pinRPWM(pin_rpwm), _pinLPWM(pin_lpwm), _pinREN(pin_ren),
	  _pinLEN(pin_len) {}

void MotorDriver::begin(uint32_t pwmFreqHz, uint8_t pwmResolutionBits) {
	pinMode(_pinREN, OUTPUT);
	pinMode(_pinLEN, OUTPUT);
	digitalWrite(_pinREN, HIGH);
	digitalWrite(_pinLEN, HIGH);

	ledcSetup(_chR, pwmFreqHz, pwmResolutionBits);
	ledcSetup(_chL, pwmFreqHz, pwmResolutionBits);
	ledcAttachPin(_pinRPWM, _chR);
	ledcAttachPin(_pinLPWM, _chL);

	_pwmMax = (1u << pwmResolutionBits) - 1u;
	stop();
}

void MotorDriver::stop() {
	ledcWrite(_chR, 0);
	ledcWrite(_chL, 0);
	_lastCmd = 0.0f;
}

void MotorDriver::setThrottle(float throttle) {
	throttle = mc::clamp(throttle, -1.0f, 1.0f);
	_lastCmd = throttle;

	const uint16_t duty = (uint16_t)(fabsf(throttle) * (float)_pwmMax);

	if (throttle >= 0.0f) {
		ledcWrite(_chL, 0);
		ledcWrite(_chR, duty);
	} else {
		ledcWrite(_chR, 0);
		ledcWrite(_chL, duty);
	}
}

} // namespace mc
