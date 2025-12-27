#include "hardware/ServoSteering.h"

namespace mc {

ServoSteering::ServoSteering(int pin) : _pin(pin) {}

void ServoSteering::begin(uint16_t minUs, uint16_t maxUs, uint16_t centerUs) {
	_minUs = minUs;
	_maxUs = maxUs;
	_centerUs = centerUs;

	ledcSetup(_ch, _freq, _resBits);
	ledcAttachPin(_pin, _ch);
	center();
}

void ServoSteering::writePulseUs(uint16_t pulseUs) {
	pulseUs = mc::clamp< uint16_t >(pulseUs, 500, 2500);
	const float duty = (float)pulseUs / (float)_periodUs;
	const uint32_t d = (uint32_t)(duty * (float)_maxDuty);
	ledcWrite(_ch, d);
}

void ServoSteering::setNormalized(float steer) {
	steer = mc::clamp(steer, -1.0f, 1.0f);
	_last = steer;

	// Map [-1..1] to [min..max]
	const float t = (steer + 1.0f) * 0.5f;
	const uint16_t us = (uint16_t)((1.0f - t) * _minUs + t * _maxUs);
	writePulseUs(us);
}

void ServoSteering::center() {
	_last = 0.0f;
	writePulseUs(_centerUs);
}

} // namespace mc
