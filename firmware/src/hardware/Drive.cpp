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

void Drive::setTargetBrake(uint8_t brake_duty, bool active, uint32_t now_ms) {
	_tgt_brake_duty = brake_duty;
	_brake_active = active;
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
	int16_t cmd_steer = (killed || expired) ? 0 : _tgt_steer_cdeg;

	if (killed || expired) {
		_applied_speed_mm_s = cmd_speed;
		_applied_pwm = 0;
		_applied_steer_cdeg = cmd_steer;
		_brake_released_at_ms = 0;
		_prev_brake_active = false;
		if (cfg::DRIVE_BRAKE_ON_KILLED && cfg::ENGINE_ACTIVE_BRAKE_ENABLE) {
			// killed/expired 時もブレーキを入れる（安全停止）
			_applied_brake_duty = cfg::DRIVE_KILL_BRAKE_DUTY;
			_engine.outputBrake(cfg::DRIVE_KILL_BRAKE_DUTY);
		} else {
			// 惰行停止
			_applied_brake_duty = 0;
			_engine.setTarget(0);
			_engine.control(dt_s);
		}
	} else if (_brake_active) {
		_applied_speed_mm_s = cmd_speed;
		_applied_pwm = 0;
		_applied_brake_duty = _tgt_brake_duty;
		_applied_steer_cdeg = cmd_steer;
		_prev_brake_active = true;
		_engine.outputBrake(
			_tgt_brake_duty); // アクティブブレーキ（逆回転ではない）
	} else {
		// ブレーキ解除直後の再加速抑制（BRAKE_COOLDOWN_MS の間は推力0）
		if (_prev_brake_active)
			_brake_released_at_ms = now_ms;
		const bool in_cooldown = _brake_released_at_ms != 0 &&
								 (uint32_t)(now_ms - _brake_released_at_ms) <=
									 cfg::BRAKE_COOLDOWN_MS;
		// cooldown
		// は「前進再加速」を抑えるのが目的。非常時の逆転パルス(負PWM)は通す。
		const int16_t pwm_to_apply =
			(in_cooldown && _tgt_pwm > 0) ? 0 : _tgt_pwm;

		_applied_speed_mm_s = cmd_speed;
		_applied_pwm = pwm_to_apply;
		_applied_brake_duty = 0;
		_applied_steer_cdeg = cmd_steer;
		if (_brake_mode) {
			_engine.setRateLimits(cfg::ENGINE_RATE_UP,
								  cfg::ENGINE_RATE_DOWN_BRAKE);
		} else {
			_engine.setRateLimits(cfg::ENGINE_RATE_UP, cfg::ENGINE_RATE_DOWN);
		}
		_engine.setTarget(pwm_to_apply);
		_engine.control(dt_s);

		if (!in_cooldown)
			_brake_released_at_ms = 0;
		_prev_brake_active = false;
	}

	_steer.setAngle(steerCdegToDeg_(cmd_steer));
}
