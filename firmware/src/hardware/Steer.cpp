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
	angle *=
		-1; // モーターの設置向きが0~1が右左となっているが,LiDARはその逆なので合わせる。
	if (ANGLE_RANGE < angle) {
		angle = ANGLE_RANGE;
	} else if (ANGLE_RANGE * -1 < angle) {
		angle = ANGLE_RANGE * -1;
	}

	angle += ANGLE_CENTER;
	int us = map(angle, 0, 180, PULSE_MIN_US, PULSE_MAX_US);
	writePulseUs_(us);
}

void Steer::center() { setAngle(ANGLE_CENTER); }
void Steer::left() { setAngle(ANGLE_CENTER + ANGLE_RANGE); }
void Steer::right() { setAngle(ANGLE_CENTER - ANGLE_RANGE); }
