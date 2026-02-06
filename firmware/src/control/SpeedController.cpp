#include "SpeedController.h"
#include "../../lib/common/Math.h"
#include "../config/Config.h"
#include <cmath>

void SpeedController::reset() {
	_i = 0.0f;
	_last_target_mm_s = 0;
}

int16_t SpeedController::speedToPwm_(int16_t v_target_mm_s) const {
	int v = (int)v_target_mm_s;
	v = mc::clamp< int >(v, -mc_config::SPEED_MAX_MM_S,
						 mc_config::SPEED_MAX_MM_S);
	long pwm = (long)v * 255L / (long)mc_config::SPEED_MAX_MM_S;
	return (int16_t)mc::clamp< long >(pwm, -255, 255);
}

SpeedControlOutput SpeedController::update(int16_t v_target_mm_s,
										   float v_est_mm_s, float dt_s,
										   bool imu_calibrated, bool active) {
	SpeedControlOutput out{};
	out.active = active;
	out.calibrated = imu_calibrated;
	out.pwm_ff = speedToPwm_(v_target_mm_s);

	if (!active) {
		reset();
		out.error_mm_s = 0.0f;
		out.integrator = _i;
		out.pwm_cmd = 0;
		return out;
	}

	const bool forward_target = (v_target_mm_s >= 0);
	const float kp =
		forward_target ? (imu_calibrated ? cfg::SPEED_KP : cfg::SPEED_KP_UNCAL)
					   : 0.0f;
	const float ki =
		forward_target ? (imu_calibrated ? cfg::SPEED_KI : cfg::SPEED_KI_UNCAL)
					   : 0.0f;

	const bool in_deadband =
		(std::abs(v_target_mm_s) < cfg::SPEED_DEADBAND_MM_S);
	const bool allow_integrator = (ki > 0.0f) && !in_deadband;

	out.error_mm_s = (float)v_target_mm_s - v_est_mm_s;

	if (v_target_mm_s == 0 ||
		((_last_target_mm_s >= 0) != (v_target_mm_s >= 0))) {
		_i = 0.0f;
	}
	_last_target_mm_s = v_target_mm_s;

	if (dt_s > 0.0f && allow_integrator) {
		_i += ki * out.error_mm_s * dt_s;
		_i = mc::clamp< float >(_i, -cfg::SPEED_I_CLAMP, cfg::SPEED_I_CLAMP);
	} else if (ki == 0.0f || in_deadband) {
		_i = 0.0f;
	}

	float pwm = (float)out.pwm_ff + kp * out.error_mm_s + _i;
	// 前進・停止時は[0,255]、逆転時は[-255,0]。逆転はABSのみが指示する
	const float pwm_lo = forward_target ? 0.0f : -255.0f;
	const float pwm_hi = forward_target ? 255.0f : 0.0f;
	float pwm_sat = mc::clamp< float >(pwm, pwm_lo, pwm_hi);
	// 前進目標かつ低速時は最小PWMを確保（静止摩擦克服）
	if (forward_target && v_target_mm_s > 0 && v_est_mm_s < 100.0f &&
		pwm_sat > 0.0f && pwm_sat < (float)cfg::SPEED_PWM_MIN_FORWARD) {
		pwm_sat = (float)cfg::SPEED_PWM_MIN_FORWARD;
	}
	out.saturated = (pwm != pwm_sat);

	if (out.saturated && allow_integrator) {
		_i = pwm_sat - (float)out.pwm_ff - kp * out.error_mm_s;
		_i = mc::clamp< float >(_i, -cfg::SPEED_I_CLAMP, cfg::SPEED_I_CLAMP);
	}

	out.integrator = _i;
	out.pwm_cmd = (int16_t)lroundf(pwm_sat);
	return out;
}
