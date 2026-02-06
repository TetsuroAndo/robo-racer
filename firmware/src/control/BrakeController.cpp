#include "BrakeController.h"
#include "../../lib/common/Math.h"
#include "../config/Config.h"
#include <algorithm>
#include <cmath>

void BrakeController::reset_() {
	_phase = Phase::IDLE;
	_phase_until_ms = 0;
	_pulse_count = 0;
	_stop_since_ms = 0;
	_last_stop_ms = 0;
	_brake_ramp = 0.0f;
	_latched_level = StopLevel::NONE;
}

BrakeControllerOutput BrakeController::update(bool stop_requested,
											  StopLevel stop_level,
											  uint16_t d_mm, float v_est_mm_s,
											  float a_brake_cap_mm_s2,
											  uint32_t now_ms) {
	BrakeControllerOutput out{};
	out.pwm_override = false;
	out.pwm_cmd = 0;
	out.brake_duty = 0;
	out.brake_active = false;
	out.brake_mode = false;

	(void)a_brake_cap_mm_s2;

	bool effective_stop = stop_requested;

	if (stop_requested) {
		_last_stop_ms = now_ms;
		_latched_level = stop_level;
	} else {
		if (_last_stop_ms != 0 &&
			(uint32_t)(now_ms - _last_stop_ms) <= cfg::BRAKE_HOLD_MS) {
			effective_stop = true;
		} else {
			reset_();
			return out;
		}
	}

	if (!effective_stop) {
		reset_();
		return out;
	}

	if (_stop_since_ms == 0)
		_stop_since_ms = now_ms;

	const StopLevel level = stop_requested ? stop_level : _latched_level;

	const auto computeBrakeDuty = [&]() -> uint8_t {
		const uint16_t stop_dist = cfg::TSD20_STOP_DISTANCE_MM;
		const float close =
			(stop_dist > 0)
				? mc::clamp< float >(1.0f - (float)d_mm / (float)stop_dist,
									 0.0f, 1.0f)
				: 1.0f;
		int pwm_max = (int)lroundf((float)cfg::BRAKE_PWM_MAX * close);
		if (pwm_max < cfg::BRAKE_PWM_MIN)
			pwm_max = cfg::BRAKE_PWM_MIN;
		if (pwm_max > cfg::BRAKE_PWM_MAX)
			pwm_max = cfg::BRAKE_PWM_MAX;
		return (uint8_t)pwm_max;
	};

	// 逆転は STOP かつ stop_requested の間だけ（MARGIN/HOLD では出さない）
	const bool allow_rev = stop_requested && (level == StopLevel::STOP);

	// STALE: 逆転せず最大短絡制動（フェイルセーフ）
	if (level == StopLevel::STALE) {
		out.brake_active = true;
		out.brake_duty = (uint8_t)cfg::BRAKE_PWM_MAX;
		out.brake_mode = true;
		out.phase = 0;
		out.pulse_count = _pulse_count;
		return out;
	}

	const int v_est_int = (int)lroundf(v_est_mm_s);

	// 低速域: 逆転禁止（後退しない）
	if (v_est_int < cfg::BRAKE_REV_EXIT_V_MM_S) {
		const uint32_t elapsed_ms = now_ms - _stop_since_ms;
		const float ramp_t = (cfg::BRAKE_RAMP_MS > 0)
								 ? (float)elapsed_ms / (float)cfg::BRAKE_RAMP_MS
								 : 1.0f;
		_brake_ramp = mc::clamp< float >(ramp_t, 0.0f, 1.0f);
		const int pwm_max = (int)computeBrakeDuty();
		const int pwm_raw = (int)lroundf((float)pwm_max * _brake_ramp);
		const int pwm_clamped =
			mc::clamp< int >(pwm_raw, cfg::BRAKE_PWM_MIN, pwm_max);

		out.brake_active = true;
		out.brake_duty = (uint8_t)pwm_clamped;
		out.brake_mode = true;
		out.phase = 0;
		out.pulse_count = _pulse_count;
		return out;
	}

	// 中速域 or MARGIN/HOLD: 逆転せず短絡制動のみ
	if (v_est_int < cfg::BRAKE_REV_MIN_V_MM_S || !allow_rev) {
		const uint32_t elapsed_ms = now_ms - _stop_since_ms;
		const float ramp_t = (cfg::BRAKE_RAMP_MS > 0)
								 ? (float)elapsed_ms / (float)cfg::BRAKE_RAMP_MS
								 : 1.0f;
		_brake_ramp = mc::clamp< float >(ramp_t, 0.0f, 1.0f);
		const int pwm_max = (int)computeBrakeDuty();
		const int pwm_raw = (int)lroundf((float)pwm_max * _brake_ramp);
		const int pwm_clamped =
			mc::clamp< int >(pwm_raw, cfg::BRAKE_PWM_MIN, pwm_max);

		out.brake_active = true;
		out.brake_duty = (uint8_t)pwm_clamped;
		out.brake_mode = true;
		out.phase = 0;
		out.pulse_count = _pulse_count;
		return out;
	}

	// パルス回数上限チェック
	if (_pulse_count >= cfg::BRAKE_REV_MAX_PULSES) {
		out.brake_active = true;
		out.brake_duty = computeBrakeDuty();
		out.brake_mode = true;
		out.phase = 0;
		out.pulse_count = _pulse_count;
		return out;
	}

	// Phase マシン: REV → COAST → (再評価) → REV or IDLE
	if (_phase == Phase::IDLE) {
		_phase = Phase::REV;
		_phase_until_ms = now_ms + cfg::BRAKE_REV_PULSE_MS;
		_pulse_count++;
	}

	if (_phase == Phase::REV) {
		if (v_est_int < cfg::BRAKE_REV_EXIT_V_MM_S) {
			_phase = Phase::IDLE;
		} else if ((int32_t)(now_ms - _phase_until_ms) >= 0) {
			_phase = Phase::COAST;
			_phase_until_ms = now_ms + cfg::BRAKE_REV_COAST_MS;
		} else {
			out.pwm_override = true;
			out.pwm_cmd = -(int16_t)cfg::BRAKE_REV_PWM;
			out.brake_mode = true;
			out.phase = 1;
			out.pulse_count = _pulse_count;
			return out;
		}
	}

	if (_phase == Phase::COAST) {
		if ((int32_t)(now_ms - _phase_until_ms) >= 0) {
			_phase = Phase::IDLE;
		} else {
			// 惰行ではなく短絡制動（距離に応じた制動）
			out.pwm_override = true;
			out.pwm_cmd = 0;
			out.brake_active = true;
			out.brake_duty = computeBrakeDuty();
			out.brake_mode = true;
			out.phase = 2;
			out.pulse_count = _pulse_count;
			return out;
		}
	}

	// IDLE に戻った直後: 次の update で REV に入る可能性
	out.brake_active = true;
	out.brake_duty = computeBrakeDuty();
	out.brake_mode = true;
	out.phase = 0;
	out.pulse_count = _pulse_count;
	return out;
}
