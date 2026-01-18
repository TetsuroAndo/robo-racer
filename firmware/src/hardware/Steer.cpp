#include "Steer.h"

namespace {
float lerp(float start, float end, float t) {
	return (1 - t) * start + t * end;
}
} // namespace

void Steer::begin() {
	ledcSetup(CH_SERVO, SERVO_FREQ, SERVO_RES);
	ledcAttachPin(PIN_SERVO, CH_SERVO);
	center();
}

void Steer::writePulseUs_(int us) {
	us = constrain(us, PULSE_MIN_US, PULSE_MAX_US);

	// 50Hz => period = 20,000us
	const uint32_t period_us = 20000;
	uint32_t duty = (uint32_t)((uint64_t)us * 65535ULL / period_us);
	ledcWrite(CH_SERVO, duty);
}

// angle -> -ANGLE_RANGE ~ ANGLE_RANGE
void Steer::setAngle(float angle) {
	// 符号はそのまま: 正が右, 負が左
	if (ANGLE_RANGE < angle) {
		angle = ANGLE_RANGE;
	} else if (angle < -ANGLE_RANGE) {
		angle = -ANGLE_RANGE;
	}

	angle += ANGLE_CENTER;
	int us = map(angle, ANGLE_CENTER - ANGLE_RANGE, ANGLE_CENTER + ANGLE_RANGE,
				 PULSE_MIN_US, PULSE_MAX_US);
	writePulseUs_(us);
}
void Steer::center() { setAngle(ANGLE_CENTER); }
void Steer::left() { setAngle(ANGLE_CENTER + ANGLE_RANGE); }
void Steer::right() { setAngle(ANGLE_CENTER - ANGLE_RANGE); }
