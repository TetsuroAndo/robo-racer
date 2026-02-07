#include "BrakeController.h"
#include "../../lib/common/Math.h"
#include "../config/Config.h"
#include <algorithm>
#include <cmath>

void BrakeController::reset_() {
	_phase = Phase::IDLE;
	_phase_until_ms = 0;
	_pulse_count = 0;
	_pulse_limit = cfg::BRAKE_REV_MAX_PULSES;
	_rev_on_ms = cfg::BRAKE_REV_PULSE_MS;
	_rev_on_ms_used = cfg::BRAKE_REV_PULSE_MS;
	_rev_v_start = 0.0f;
	_stop_since_ms = 0;
	_last_stop_ms = 0;
	_brake_ramp = 0.0f;
	_latched_level = StopLevel::NONE;
	_prev_stop_requested = false;
}

void BrakeController::fillDebug_(BrakeControllerOutput &out,
								 uint8_t phase) const {
	out.phase = phase;
	out.rev_on_ms = _rev_on_ms;
	out.pulse_count = _pulse_count;
	out.pulse_limit = _pulse_limit;
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

	bool effective_stop = stop_requested;

	if (stop_requested && !_prev_stop_requested) {
		// stop_requested検出時に内部状態をリセット（立ち上がりエッジのみ）
		reset_();
		_last_stop_ms = now_ms;
		_latched_level = stop_level;
	} else if (stop_requested) {
		// stop_requestedが継続中はリセットしない
		_last_stop_ms = now_ms;
		_latched_level = stop_level;
	} else {
		if (_last_stop_ms != 0 &&
			(uint32_t)(now_ms - _last_stop_ms) <= cfg::BRAKE_HOLD_MS) {
			effective_stop = true;
		} else {
			reset_();
			fillDebug_(out, 0);
			return out;
		}
	}

	if (!effective_stop) {
		reset_();
		fillDebug_(out, 0);
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

	// HOLD/MARGIN に落ちたら REV は即中止（後退リスクを締める）
	if (!allow_rev && _phase == Phase::REV) {
		_phase = Phase::COAST;
		_phase_until_ms = now_ms + cfg::BRAKE_REV_COAST_MS;
	}

	// STALE: 逆転せず最大短絡制動（フェイルセーフ）
	if (level == StopLevel::STALE) {
		out.brake_active = true;
		out.brake_duty = (uint8_t)cfg::BRAKE_PWM_MAX;
		out.brake_mode = true;
		fillDebug_(out, 0);
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
		fillDebug_(out, 0);
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
		fillDebug_(out, 0);
		return out;
	}

	// パルス回数上限チェック（_pulse_limit は適応で調整）
	if (_pulse_limit == 0)
		_pulse_limit = cfg::BRAKE_REV_MAX_PULSES;
	if (_pulse_count >= _pulse_limit) {
		out.brake_active = true;
		out.brake_duty = computeBrakeDuty();
		out.brake_mode = true;
		fillDebug_(out, 0);
		return out;
	}

	// Phase マシン: REV → COAST → (再評価) → REV or IDLE
	if (_phase == Phase::IDLE) {
		if (allow_rev) {
			_phase = Phase::REV;
			_rev_on_ms_used = (uint16_t)mc::clamp< int >(
				(int)_rev_on_ms, cfg::BRAKE_REV_ON_MIN_MS,
				cfg::BRAKE_REV_ON_MAX_MS);
			_phase_until_ms = now_ms + _rev_on_ms_used;
			_rev_v_start = v_est_mm_s;
			_pulse_count++;
		}
	}

	if (_phase == Phase::REV) {
		if (v_est_int < cfg::BRAKE_REV_EXIT_V_MM_S) {
			_phase = Phase::IDLE;
		} else if ((int32_t)(now_ms - _phase_until_ms) >= 0) {
			// REV終了：実測で"効き"評価して次を調整
			const float v_start = std::max(0.0f, _rev_v_start);
			const float v_end = std::max(0.0f, v_est_mm_s);
			const float dv = std::max(0.0f, v_start - v_end);
			const float t_rev_s =
				std::max(0.001f, (float)_rev_on_ms_used * 0.001f);
			const float a_eff = dv / t_rev_s;

			const int d_eff_i =
				std::max< int >((int)cfg::BRAKE_STOP_MARGIN_MM,
								(int)d_mm - (int)cfg::BRAKE_STOP_MARGIN_MM);
			const float d_eff = (float)d_eff_i;
			float a_req = (v_start * v_start) / (2.0f * d_eff);
			float a_tgt = std::max(a_req, cfg::BRAKE_REV_A_TGT_FLOOR_MM_S2);
			if (a_brake_cap_mm_s2 > 100.0f) {
				a_tgt = std::min(a_tgt, a_brake_cap_mm_s2);
			}

			const int dv_i = (int)lroundf(dv);
			if (dv_i >= cfg::BRAKE_REV_MIN_DV_MM_S) {
				const float r = (a_tgt > 1.0f) ? (a_eff / a_tgt) : 1.0f;
				out.dv_mm_s = dv;
				out.a_eff_mm_s2 = a_eff;
				out.a_tgt_mm_s2 = a_tgt;
				out.r_eff = r;
				out.adapt_event = true;
				if (r < cfg::BRAKE_REV_EFF_LOW) {
					_rev_on_ms = (uint16_t)std::min< int >(
						(int)_rev_on_ms + cfg::BRAKE_REV_ON_STEP_MS,
						cfg::BRAKE_REV_ON_MAX_MS);
				} else if (r > cfg::BRAKE_REV_EFF_HIGH) {
					_rev_on_ms = (uint16_t)std::max< int >(
						(int)_rev_on_ms - cfg::BRAKE_REV_ON_STEP_MS,
						cfg::BRAKE_REV_ON_MIN_MS);
				}
				const float dv_need =
					std::max(0.0f, v_end - (float)cfg::BRAKE_REV_EXIT_V_MM_S);
				const int remain =
					(dv_need > 0.0f) ? (int)std::ceil(dv_need / dv) : 0;
				const int limit = (int)_pulse_count + remain;
				_pulse_limit = (uint8_t)mc::clamp< int >(
					limit, 1, cfg::BRAKE_REV_MAX_PULSES);
			} else {
				// dv が見えない＝効き不明 → 最大許可寄りに倒す
				_pulse_limit = cfg::BRAKE_REV_MAX_PULSES;
			}

			_phase = Phase::COAST;
			_phase_until_ms = now_ms + cfg::BRAKE_REV_COAST_MS;
		} else {
			out.pwm_override = true;
			out.pwm_cmd = -(int16_t)cfg::BRAKE_REV_PWM;
			out.brake_mode = true;
			fillDebug_(out, 1);
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
			fillDebug_(out, 2);
			return out;
		}
	}

	// IDLE に戻った直後: 次の update で REV に入る可能性
	out.brake_active = true;
	out.brake_duty = computeBrakeDuty();
	out.brake_mode = true;
	fillDebug_(out, 0);
	_prev_stop_requested = stop_requested;
	return out;
}
