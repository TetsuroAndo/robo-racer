#include "Steer.h"
#include "../config/Config.h"

namespace {
float lerp(float start, float end, float t) {
	return (1 - t) * start + t * end;
}
} // namespace

void Steer::begin() {
	ledcSetup(cfg::STEER_CHANNEL, cfg::STEER_PWM_FREQ_HZ,
			  cfg::STEER_PWM_RESOLUTION_BITS);
	ledcAttachPin(cfg::STEER_PIN_SERVO, cfg::STEER_CHANNEL);
	center();
}

void Steer::writePulseUs_(int us) {
	us = constrain(us, cfg::STEER_PULSE_MIN_US, cfg::STEER_PULSE_MAX_US);

	// PWM周期（50Hzなら20,000us）
	const uint32_t period_us = cfg::STEER_PWM_PERIOD_US;
	uint32_t duty = (uint32_t)((uint64_t)us * 65535ULL / period_us);
	ledcWrite(cfg::STEER_CHANNEL, duty);
}

// angle -> -ANGLE_RANGE ~ ANGLE_RANGE
void Steer::setAngle(float angle) {
	// 符号はそのまま: 正が右, 負が左
	if (cfg::STEER_ANGLE_RANGE_DEG < angle) {
		angle = cfg::STEER_ANGLE_RANGE_DEG;
	} else if (angle < -cfg::STEER_ANGLE_RANGE_DEG) {
		angle = -cfg::STEER_ANGLE_RANGE_DEG;
	}

	angle += cfg::STEER_ANGLE_CENTER_DEG;
	int us =
		map(angle, cfg::STEER_ANGLE_CENTER_DEG - cfg::STEER_ANGLE_RANGE_DEG,
			cfg::STEER_ANGLE_CENTER_DEG + cfg::STEER_ANGLE_RANGE_DEG,
			cfg::STEER_PULSE_MIN_US, cfg::STEER_PULSE_MAX_US);
	writePulseUs_(us);
}
void Steer::center() { setAngle(cfg::STEER_ANGLE_CENTER_DEG); }
void Steer::left() {
	setAngle(cfg::STEER_ANGLE_CENTER_DEG + cfg::STEER_ANGLE_RANGE_DEG);
}
void Steer::right() {
	setAngle(cfg::STEER_ANGLE_CENTER_DEG - cfg::STEER_ANGLE_RANGE_DEG);
}
