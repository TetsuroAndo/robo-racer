#include "ImuEstimator.h"
#include "../config/Config.h"
#include <algorithm>
#include <cmath>

namespace {
static constexpr float kG_mm_s2 = 9806.65f;
}

void ImuEstimator::reset(uint32_t now_ms) {
	_st = ImuEstimate{};
	_calib_started = false;
	_calib_start_ms = now_ms;
	_sum_ax = _sum_ay = _sum_az = _sum_gz = 0;
	_sum_n = 0;
	_bias_ax = _bias_ay = _bias_az = _bias_gz = 0;
	_zupt_ms = 0;
	_st.a_brake_cap_mm_s2 = (float)cfg::IMU_BRAKE_INIT_MM_S2;
}

void ImuEstimator::update(const ImuSample &s, uint32_t now_ms,
						  int16_t applied_speed_mm_s,
						  int16_t target_speed_mm_s) {
	if (!_calib_started) {
		_calib_started = true;
		_calib_start_ms = now_ms;
	}

	updateBias_(s, now_ms);

	const float accel_scale = accelScaleMmS2PerLsb_();
	const float gyro_scale = gyroScaleDpsPerLsb_();

	const int32_t ax_raw = (int32_t)s.ax - _bias_ax;
	const int32_t ay_raw = (int32_t)s.ay - _bias_ay;
	const int32_t az_raw = (int32_t)s.az - _bias_az;
	const int32_t gz_raw = (int32_t)s.gz - _bias_gz;

	const float ax = (float)(ax_raw * cfg::IMU_AXIS_SIGN_X) * accel_scale;
	const float ay = (float)(ay_raw * cfg::IMU_AXIS_SIGN_Y) * accel_scale;
	(void)ay;
	const float az = (float)(az_raw * cfg::IMU_AXIS_SIGN_Z) * accel_scale;
	(void)az;
	const float gz = (float)(gz_raw * cfg::IMU_AXIS_SIGN_Z) * gyro_scale;

	_st.a_long_mm_s2 = ax;
	_st.gz_dps = gz;

	if (_st.last_ms == 0) {
		_st.last_ms = now_ms;
		return;
	}

	const uint32_t dt_ms = now_ms - _st.last_ms;
	_st.last_ms = now_ms;
	const float dt_s = std::max(0.0f, std::min(0.1f, dt_ms / 1000.0f));

	float v = _st.v_est_mm_s + _st.a_long_mm_s2 * dt_s;
	const float leak = cfg::IMU_V_EST_LEAK_PER_S * dt_s;
	if (leak > 0.0f) {
		const float scale = std::max(0.0f, 1.0f - leak);
		v *= scale;
	}

	const bool zupt_cond =
		std::fabs(_st.a_long_mm_s2) < (float)cfg::IMU_ZUPT_A_LONG_MM_S2 &&
		std::fabs(_st.gz_dps) < cfg::IMU_ZUPT_GZ_DPS &&
		std::abs(applied_speed_mm_s) < cfg::IMU_ZUPT_SPEED_MM_S;

	if (zupt_cond) {
		_zupt_ms += dt_ms;
	} else {
		_zupt_ms = 0;
	}

	if (_zupt_ms >= cfg::IMU_ZUPT_HOLD_MS) {
		v = 0.0f;
	}

	if (v < 0.0f)
		v = 0.0f;
	if (v > (float)cfg::IMU_V_EST_MAX_MM_S)
		v = (float)cfg::IMU_V_EST_MAX_MM_S;

	_st.v_est_mm_s = v;

	const float decel = std::max(0.0f, -_st.a_long_mm_s2);
	const bool braking = (applied_speed_mm_s > 0) &&
						 ((applied_speed_mm_s - target_speed_mm_s) >
							  cfg::IMU_BRAKE_CMD_DELTA_MM_S ||
						  decel > (float)cfg::IMU_BRAKE_DETECT_MM_S2);
	if (braking && decel > 0.0f) {
		float cap = _st.a_brake_cap_mm_s2;
		const float min_cap = (float)cfg::IMU_BRAKE_MIN_MM_S2;
		const float max_cap = (float)cfg::IMU_BRAKE_MAX_MM_S2;
		if (cap <= 0.0f)
			cap = (float)cfg::IMU_BRAKE_INIT_MM_S2;
		const float alpha =
			(decel < cap) ? cfg::IMU_BRAKE_ALPHA_DOWN : cfg::IMU_BRAKE_ALPHA_UP;
		cap = (1.0f - alpha) * cap + alpha * decel;
		if (cap < min_cap)
			cap = min_cap;
		else if (cap > max_cap)
			cap = max_cap;
		_st.a_brake_cap_mm_s2 = cap;
	}
}

void ImuEstimator::updateBias_(const ImuSample &s, uint32_t now_ms) {
	if (_st.calibrated)
		return;
	if ((now_ms - _calib_start_ms) > cfg::IMU_CALIBRATION_MS && _sum_n > 0) {
		_bias_ax = (int32_t)(_sum_ax / (int64_t)_sum_n);
		_bias_ay = (int32_t)(_sum_ay / (int64_t)_sum_n);
		_bias_az = (int32_t)(_sum_az / (int64_t)_sum_n);
		_bias_gz = (int32_t)(_sum_gz / (int64_t)_sum_n);
		_st.calibrated = true;
		return;
	}

	_sum_ax += s.ax;
	_sum_ay += s.ay;
	_sum_az += s.az;
	_sum_gz += s.gz;
	_sum_n += 1;
}

float ImuEstimator::accelScaleMmS2PerLsb_() const {
	int lsb_per_g = 4096;
	switch (cfg::IMU_ACCEL_FS_SEL & 0x03) {
	case 0:
		lsb_per_g = 16384;
		break;
	case 1:
		lsb_per_g = 8192;
		break;
	case 2:
		lsb_per_g = 4096;
		break;
	case 3:
		lsb_per_g = 2048;
		break;
	}
	return kG_mm_s2 / (float)lsb_per_g;
}

float ImuEstimator::gyroScaleDpsPerLsb_() const {
	float lsb_per_dps = 16.4f;
	switch (cfg::IMU_GYRO_FS_SEL & 0x03) {
	case 0:
		lsb_per_dps = 131.0f;
		break;
	case 1:
		lsb_per_dps = 65.5f;
		break;
	case 2:
		lsb_per_dps = 32.8f;
		break;
	case 3:
		lsb_per_dps = 16.4f;
		break;
	}
	return 1.0f / lsb_per_dps;
}
