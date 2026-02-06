#include "Drive.h"
#include "../../lib/common/Math.h"
#include "../config/Config.h"

void Drive::begin() {
	_engine.begin();
	_steer.begin();
	_lastUpdateMs = millis();
}

void Drive::setTargetMmS(int16_t speed_mm_s, uint32_t now_ms) {
	_tgt_speed_mm_s = speed_mm_s;
	_lastUpdateMs = now_ms;
}

void Drive::setTargetPwm(int16_t speed_pwm, uint32_t now_ms) {
	_tgt_pwm = speed_pwm;
	_lastUpdateMs = now_ms;
}

void Drive::setTargetSteerCdeg(int16_t steer_cdeg, uint32_t now_ms) {
	_tgt_steer_cdeg = steer_cdeg;
	_lastUpdateMs = now_ms;
}

void Drive::setTtlMs(uint16_t ttl_ms, uint32_t now_ms) {
	_ttl_ms = ttl_ms;
	_lastUpdateMs = now_ms;
}

void Drive::setDistMm(uint16_t dist_mm, uint32_t now_ms) {
	_dist_mm = dist_mm;
	_lastUpdateMs = now_ms;
}

void Drive::setBrakeMode(bool enabled) { _brake_mode = enabled; }

float Drive::steerCdegToDeg_(int16_t cdeg) const {
	float deg = (float)cdeg / 100.0f;
	return mc::clamp< float >(deg, mc_config::STEER_ANGLE_MIN_DEG,
							  mc_config::STEER_ANGLE_MAX_DEG);
}

void Drive::tick(uint32_t now_ms, float dt_s, bool killed) {
	const uint32_t age_ms = now_ms - _lastUpdateMs;
	const bool expired = (age_ms > (uint32_t)_ttl_ms);

	int16_t cmd_speed = (killed || expired) ? 0 : _tgt_speed_mm_s;
	int16_t cmd_pwm = (killed || expired) ? 0 : _tgt_pwm;
	int16_t cmd_steer = (killed || expired) ? 0 : _tgt_steer_cdeg;

	_applied_speed_mm_s = cmd_speed;
	_applied_pwm = cmd_pwm;
	_applied_steer_cdeg = cmd_steer;

	// SlewRateLimiter expects per-second rates.
	if (_brake_mode) {
		_engine.setRateLimits(cfg::ENGINE_RATE_UP, cfg::ENGINE_RATE_DOWN_BRAKE);
	} else {
		_engine.setRateLimits(cfg::ENGINE_RATE_UP, cfg::ENGINE_RATE_DOWN);
	}
	_engine.setTarget(cmd_pwm);
	_engine.control(dt_s);

	_steer.setAngle(steerCdegToDeg_(cmd_steer));
}
