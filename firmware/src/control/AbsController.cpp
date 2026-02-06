#include "AbsController.h"
#include "../../lib/common/Math.h"
#include "../config/Config.h"
#include <algorithm>
#include <cmath>

void AbsController::reset(uint32_t now_ms) {
	_active = false;
	_cycle_ms = now_ms;
	_duty = 0.0f;
	_i = 0.0f;
	_hold_until_ms = 0;
}

int16_t AbsController::apply(uint32_t now_ms, float dt_s, int16_t speed_mm_s,
							 bool allow_abs, const ImuEstimate &imu,
							 bool imu_valid, const Tsd20State &tsd,
							 AbsDiag *diag, bool *active_out) {
	AbsDiag local{};
	AbsDiag *d = diag ? diag : &local;
	*d = AbsDiag{};
	d->reason = ABS_DISABLED;
	d->v_cmd = (float)speed_mm_s;
	d->dt_s = dt_s;

	if (active_out)
		*active_out = false;

	if (!cfg::ABS_ENABLE || !allow_abs) {
		reset(now_ms);
		return speed_mm_s;
	}

	if (!imu_valid) {
		d->reason = ABS_NO_IMU;
		reset(now_ms);
		return speed_mm_s;
	}

	if (cfg::ABS_REQUIRE_CALIB && !imu.calibrated) {
		d->reason = ABS_NOT_CALIB;
		reset(now_ms);
		return speed_mm_s;
	}

	if (speed_mm_s < 0) {
		d->reason = ABS_REVERSE;
		reset(now_ms);
		return speed_mm_s;
	}

	const float v_cmd = std::max(0.0f, (float)speed_mm_s);
	const float v_est = std::max(0.0f, imu.v_est_mm_s);
	d->v_cmd = v_cmd;
	d->v_est = v_est;
	const float margin = (float)cfg::ABS_SPEED_MARGIN_MM_S;
	const bool want_brake = (v_est > (v_cmd + margin));

	if (want_brake) {
		_hold_until_ms = now_ms + cfg::ABS_BRAKE_HOLD_MS;
	}
	const bool active = want_brake || (_active && (_hold_until_ms != 0) &&
									   (now_ms <= _hold_until_ms));

	if (!active) {
		d->reason = ABS_INACTIVE;
		reset(now_ms);
		return speed_mm_s;
	}

	if (!_active) {
		_active = true;
		_cycle_ms = now_ms;
		_duty = (float)cfg::ABS_DUTY_MIN / 100.0f;
		_i = 0.0f;
	}

	float a_cap = imu.a_brake_cap_mm_s2;
	if (a_cap <= 0.0f)
		a_cap = (float)cfg::IMU_BRAKE_INIT_MM_S2;
	if (a_cap < (float)cfg::IMU_BRAKE_MIN_MM_S2)
		a_cap = (float)cfg::IMU_BRAKE_MIN_MM_S2;
	if (a_cap > (float)cfg::IMU_BRAKE_MAX_MM_S2)
		a_cap = (float)cfg::IMU_BRAKE_MAX_MM_S2;

	float dv = v_est - v_cmd;
	if (dv < 0.0f)
		dv = 0.0f;
	float t_brake = (float)cfg::ABS_BRAKE_HOLD_MS / 1000.0f;
	if (t_brake < 0.05f)
		t_brake = 0.05f;
	float a_target = (dv / t_brake) * cfg::ABS_A_TARGET_SCALE;
	if (a_target > a_cap)
		a_target = a_cap;
	if (a_target < 0.0f)
		a_target = 0.0f;

	const float decel = std::max(0.0f, -imu.a_long_mm_s2);
	const float e = a_target - decel;
	d->a_target = a_target;
	d->a_cap = a_cap;
	d->decel = decel;
	d->reason = ABS_ACTIVE;

	_i += e * cfg::ABS_DUTY_KI * dt_s;
	float duty = _duty + cfg::ABS_DUTY_KP * e + _i;

	const float duty_min = (float)cfg::ABS_DUTY_MIN / 100.0f;
	const float duty_max = (float)cfg::ABS_DUTY_MAX / 100.0f;
	duty = mc::clamp< float >(duty, duty_min, duty_max);
	_duty = duty;
	d->duty = duty;
	d->active = true;
	if (active_out)
		*active_out = true;

	if (cfg::ABS_PERIOD_MS == 0) {
		return speed_mm_s;
	}

	const uint32_t period = cfg::ABS_PERIOD_MS;
	if ((uint32_t)(now_ms - _cycle_ms) >= period) {
		const uint32_t elapsed = (uint32_t)(now_ms - _cycle_ms);
		_cycle_ms = now_ms - (elapsed % period);
	}

	const uint32_t phase = (uint32_t)(now_ms - _cycle_ms);
	const uint32_t duty_ms = (uint32_t)lroundf((float)period * duty);
	const bool reverse_on = (phase < duty_ms);

	if (cfg::TSD20_ENABLE && tsd.valid) {
		const uint16_t limit = (uint16_t)(cfg::TSD20_MARGIN_MM +
										  cfg::ABS_REVERSE_DISABLE_MARGIN_MM);
		if (tsd.mm <= limit) {
			return 0;
		}
	}

	const int max_mm_s = mc_config::SPEED_MAX_MM_S;
	int reverse_mm_s = cfg::ABS_REVERSE_MM_S;
	if (reverse_mm_s > max_mm_s)
		reverse_mm_s = max_mm_s;
	if (reverse_mm_s < 0)
		reverse_mm_s = -reverse_mm_s;

	return reverse_on ? (int16_t)(-reverse_mm_s) : 0;
}
