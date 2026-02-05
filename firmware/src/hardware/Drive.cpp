#include "Drive.h"
#include "../../lib/common/Math.h"

void Drive::begin() {
	_engine.begin();
	_steer.begin();
	_lastUpdateMs = millis();
}

void Drive::setTargetMmS(int16_t speed_mm_s) {
	_tgt_speed_mm_s = speed_mm_s;
	_lastUpdateMs = millis();
}

void Drive::setTargetSteerCdeg(int16_t steer_cdeg) {
	_tgt_steer_cdeg = steer_cdeg;
	_lastUpdateMs = millis();
}

void Drive::setTtlMs(uint16_t ttl_ms) {
	_ttl_ms = ttl_ms;
	_lastUpdateMs = millis();
}

void Drive::setDistMm(uint16_t dist_mm) {
	_dist_mm = dist_mm;
	_lastUpdateMs = millis();
}

int Drive::speedMmSToPwm_(int16_t mm_s) const {
	int v = (int)mm_s;
	v = mc::clamp< int >(v, -MAX_SPEED_MM_S, MAX_SPEED_MM_S);
	long pwm = (long)v * 255L / (long)MAX_SPEED_MM_S;
	return (int)mc::clamp< long >(pwm, -255, 255);
}

float Drive::steerCdegToDeg_(int16_t cdeg) const {
	float deg = (float)cdeg / 100.0f;
	return mc::clamp< float >(deg, cfg::STEER_ANGLE_MIN_DEG,
							  cfg::STEER_ANGLE_MAX_DEG);
}

void Drive::tick(uint32_t now_ms, float dt_s, bool killed) {
	bool expired = ((uint32_t)(now_ms - _lastUpdateMs) > (uint32_t)_ttl_ms);

	int16_t cmd_speed = (killed || expired) ? 0 : _tgt_speed_mm_s;
	int16_t cmd_steer = (killed || expired) ? 0 : _tgt_steer_cdeg;

	_applied_speed_mm_s = cmd_speed;
	_applied_steer_cdeg = cmd_steer;

	_engine.setTarget(speedMmSToPwm_(cmd_speed));
	_engine.control(dt_s);

	_steer.setAngle(steerCdegToDeg_(cmd_steer));
}
