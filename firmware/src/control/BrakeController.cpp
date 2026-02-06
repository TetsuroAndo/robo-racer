#include "BrakeController.h"
#include "../../lib/common/Math.h"
#include "../config/Config.h"
#include <algorithm>
#include <cmath>

BrakeControllerOutput BrakeController::update(bool stop_requested,
											  StopLevel stop_level,
											  uint16_t d_mm, float v_est_mm_s,
											  uint32_t now_ms) {
	BrakeControllerOutput out{};
	out.active = false;
	out.brake_duty = 0;

	// (B) 止まれ要求の時だけ負PWMを許可
	// HOLD: STOP が一瞬途切れても BRAKE_HOLD_MS はブレーキ維持
	bool effective_stop = stop_requested;
	if (stop_requested) {
		_last_stop_ms = now_ms;
	} else {
		if (_last_stop_ms != 0 &&
			(uint32_t)(now_ms - _last_stop_ms) <= cfg::BRAKE_HOLD_MS) {
			effective_stop = true;
		} else {
			_stop_since_ms = 0;
			_last_stop_ms = 0;
			_brake_ramp = 0.0f;
			return out;
		}
	}

	if (!effective_stop) {
		_stop_since_ms = 0;
		_brake_ramp = 0.0f;
		return out;
	}

	if (_stop_since_ms == 0)
		_stop_since_ms = now_ms;

	// (A) 動いてる時だけ負PWMを許可
	if (v_est_mm_s <= (float)cfg::BRAKE_V_EPS_MM_S) {
		_brake_ramp = 0.0f;
		return out;
	}

	// (C) 壁が近いほど弱くする（後退開始を防ぐ）
	// d_mm: 0..STOP_DIST -> pwm_max: 0..BRAKE_PWM_MAX。d_mm=0 ではブレーキ禁止
	const uint16_t stop_dist =
		(cfg::TSD20_STOP_DISTANCE_MM > 0) ? cfg::TSD20_STOP_DISTANCE_MM : 500u;
	const float ratio = (stop_dist > 0 && d_mm < stop_dist)
							? (float)d_mm / (float)stop_dist
							: 1.0f;
	int pwm_max_allowed = (int)lroundf((float)cfg::BRAKE_PWM_MAX *
									   mc::clamp< float >(ratio, 0.0f, 1.0f));
	// (D) STOP/MARGIN/STALE で duty 上限を段階化（MARGIN は弱め、STALE は最小）
	switch (stop_level) {
	case StopLevel::STOP:
		break; // そのまま
	case StopLevel::MARGIN:
		pwm_max_allowed = std::min(pwm_max_allowed, cfg::BRAKE_PWM_MAX / 2);
		break;
	case StopLevel::STALE:
		pwm_max_allowed = std::min(pwm_max_allowed, cfg::BRAKE_PWM_MIN);
		break;
	default:
		break;
	}
	// 壁際（pwm_max_allowed < MIN）ではブレーキ出さない
	const int pwm_max = (pwm_max_allowed < cfg::BRAKE_PWM_MIN)
							? 0
							: std::min(pwm_max_allowed, cfg::BRAKE_PWM_MAX);
	if (pwm_max == 0) {
		_brake_ramp = 0.0f;
		return out;
	}

	// ランプアップ（BRAKE_RAMP_MS で目標値に到達）
	const uint32_t elapsed_ms = now_ms - _stop_since_ms;
	const float ramp_t = (cfg::BRAKE_RAMP_MS > 0)
							 ? (float)elapsed_ms / (float)cfg::BRAKE_RAMP_MS
							 : 1.0f;
	_brake_ramp = mc::clamp< float >(ramp_t, 0.0f, 1.0f);

	const int pwm_raw = (int)lroundf((float)pwm_max * _brake_ramp);
	const int pwm_clamped =
		mc::clamp< int >(pwm_raw, cfg::BRAKE_PWM_MIN, pwm_max);

	out.active = true;
	out.brake_duty = (uint8_t)pwm_clamped;
	return out;
}
