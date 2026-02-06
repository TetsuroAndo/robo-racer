#include "DecelController.h"
#include "../../lib/common/Math.h"
#include "../config/Config.h"
#include <cmath>

void DecelController::reset() { _u_i = 0.0f; }

DecelControlOutput DecelController::update(int16_t v_target_mm_s,
										   float v_est_mm_s,
										   float a_long_lpf_mm_s2,
										   float a_brake_cap_mm_s2, float dt_s,
										   bool imu_calib, bool enable) {
	DecelControlOutput out;

	if (!enable) {
		reset();
		return out;
	}
	if (!imu_calib) {
		// 既存方針に合わせ、IMU未校正時は能動制動しない（=安全側/再現性優先）
		reset();
		return out;
	}
	if (dt_s <= 0.0f) {
		return out;
	}

	const float v_tgt = (v_target_mm_s < 0) ? 0.0f : (float)v_target_mm_s;
	const float v_err = v_est_mm_s - v_tgt;

	// 減速が不要なら何もしない
	if (v_err <= (float)cfg::DECEL_V_ERR_DEADBAND_MM_S) {
		reset();
		return out;
	}

	// 停止付近はブレーキを抜いて後退を抑止
	if (v_est_mm_s <= (float)cfg::DECEL_STOP_EPS_MM_S) {
		reset();
		return out;
	}

	const float a_cap =
		mc::clamp(a_brake_cap_mm_s2, (float)cfg::DECEL_A_CAP_FLOOR_MM_S2,
				  (float)cfg::DECEL_A_CAP_MAX_MM_S2);

	// a_tgt = -(v_err / tau) をベースに、最小/最大で拘束
	float a_tgt = -(v_err / cfg::DECEL_TAU_S);
	a_tgt = -mc::clamp(-a_tgt, (float)cfg::DECEL_A_MIN_MM_S2, a_cap);

	const float u_ff = -(float)cfg::DECEL_PWM_MAX_BRAKE * (-a_tgt) / a_cap;
	const float a_err = a_tgt - a_long_lpf_mm_s2;

	_u_i += cfg::DECEL_KI * a_err * dt_s;
	_u_i = mc::clamp(_u_i, -(float)cfg::DECEL_PWM_MAX_BRAKE, 0.0f);

	const float u_fb = cfg::DECEL_KP * a_err + _u_i;
	float u = u_ff + u_fb;
	u = mc::clamp(u, -(float)cfg::DECEL_PWM_MAX_BRAKE, 0.0f);

	out.pwm_cmd = (int16_t)u;
	out.a_tgt_mm_s2 = a_tgt;
	out.a_err_mm_s2 = a_err;
	out.u_ff = u_ff;
	out.u_fb = u_fb;
	out.u_i = _u_i;
	out.active = true;
	return out;
}
