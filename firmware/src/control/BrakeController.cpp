#include "BrakeController.h"
#include "../../lib/common/Math.h"
#include "../config/Config.h"
#include <algorithm>
#include <cmath>

BrakeControllerOutput BrakeController::update(bool stop_requested,
											  StopLevel stop_level,
											  uint16_t d_mm, float v_est_mm_s,
											  bool v_est_valid,
											  uint32_t now_ms) {
	BrakeControllerOutput out{};
	out.active = false;
	out.brake_duty = 0;

	bool effective_stop = stop_requested;

	if (stop_requested) {
		_last_stop_ms = now_ms;
		_latched_level = stop_level;
	} else {
		if (_last_stop_ms != 0 &&
			(uint32_t)(now_ms - _last_stop_ms) <= cfg::BRAKE_HOLD_MS) {
			effective_stop = true;
		} else {
			_stop_since_ms = 0;
			_last_stop_ms = 0;
			_brake_ramp = 0.0f;
			_latched_level = StopLevel::NONE;
			return out;
		}
	}

	if (!effective_stop) {
		_stop_since_ms = 0;
		_brake_ramp = 0.0f;
		_latched_level = StopLevel::NONE;
		return out;
	}

	if (_stop_since_ms == 0)
		_stop_since_ms = now_ms;

	// NOTE: v_est が怪しい/無い状況でも「止めたい」ならブレーキは出す。
	// ここで v_est を根拠にブレーキを止めると、IMU不調時に惰行して突っ込む。
	(void)v_est_valid;
	(void)d_mm;
	(void)v_est_mm_s;

	// stop_level は stop_requested が落ちた後も HOLD 中は NONE
	// になり得るのでラッチを使う
	const StopLevel level = stop_requested ? stop_level : _latched_level;

	int pwm_max = 0;
	switch (level) {
	case StopLevel::STOP:
		pwm_max = cfg::BRAKE_PWM_MAX;
		break;
	case StopLevel::MARGIN:
		pwm_max = cfg::BRAKE_PWM_MAX / 2;
		break;
	case StopLevel::STALE:
		// stale は "安全側に倒す"＝止める
		pwm_max = cfg::BRAKE_PWM_MAX;
		break;
	default:
		// HOLD で入ってきた場合など
		pwm_max = cfg::BRAKE_PWM_MAX / 2;
		break;
	}

	// active にするなら最低 duty は確保する（0
	// になると「近いほど効かない」を再発する）
	if (pwm_max < cfg::BRAKE_PWM_MIN)
		pwm_max = cfg::BRAKE_PWM_MIN;

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
