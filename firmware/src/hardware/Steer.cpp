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

// angle -> STEER_ANGLE_MIN_DEG ~ STEER_ANGLE_MAX_DEG
void Steer::setAngle(float angle) {
	// 符号はそのまま: 正が右, 負が左
	if (mc_config::STEER_ANGLE_MAX_DEG < angle) {
		angle = mc_config::STEER_ANGLE_MAX_DEG;
	} else if (angle < mc_config::STEER_ANGLE_MIN_DEG) {
		angle = mc_config::STEER_ANGLE_MIN_DEG;
	}

	const float min_angle =
		cfg::STEER_ANGLE_CENTER_DEG + mc_config::STEER_ANGLE_MIN_DEG;
	const float max_angle =
		cfg::STEER_ANGLE_CENTER_DEG + mc_config::STEER_ANGLE_MAX_DEG;
	const float angle_abs = angle + cfg::STEER_ANGLE_CENTER_DEG;
	float t = (angle_abs - min_angle) / (max_angle - min_angle);
	if (t < 0.0f)
		t = 0.0f;
	if (t > 1.0f)
		t = 1.0f;
	const int us = (int)lerp((float)cfg::STEER_PULSE_MIN_US,
							 (float)cfg::STEER_PULSE_MAX_US, t);
	writePulseUs_(us);
}
void Steer::center() { setAngle(0.0f); }
