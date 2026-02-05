#include "ImuEstimator.h"
#include "../config/Config.h"
#include <algorithm>
#include <cmath>

namespace {
static constexpr float kG_mm_s2 = 9806.65f;

static inline int clampAxisIndex(int idx) {
	if (idx < 0)
		return 0;
	if (idx > 2)
		return 2;
	return idx;
}

static inline int32_t mapAxis(const int32_t v[3], int map, int sign) {
	const int idx = clampAxisIndex(map);
	return v[idx] * (int32_t)sign;
}
} // namespace

void ImuEstimator::reset(uint32_t now_ms) {
	_st = ImuEstimate{};
	_calib_started = false;
	_calib_start_ms = now_ms;
	_sum_ax = _sum_ay = _sum_az = _sum_gx = _sum_gy = _sum_gz = 0;
	_sum_n = 0;
	_bias_ax = _bias_ay = _bias_az = _bias_gx = _bias_gy = _bias_gz = 0;
	_zupt_ms = 0;
	_g_est_x = 0.0f;
	_g_est_y = 0.0f;
	_g_est_z = 0.0f;
	_gravity_init = false;
	_fusion_init = false;
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

	const int32_t accel_raw[3] = {(int32_t)s.ax - _bias_ax,
								  (int32_t)s.ay - _bias_ay,
								  (int32_t)s.az - _bias_az};
	const int32_t gyro_raw[3] = {(int32_t)s.gx - _bias_gx,
								 (int32_t)s.gy - _bias_gy,
								 (int32_t)s.gz - _bias_gz};

	const int32_t ax_raw =
		mapAxis(accel_raw, cfg::IMU_AXIS_MAP_X, cfg::IMU_AXIS_SIGN_X);
	const int32_t ay_raw =
		mapAxis(accel_raw, cfg::IMU_AXIS_MAP_Y, cfg::IMU_AXIS_SIGN_Y);
	const int32_t az_raw =
		mapAxis(accel_raw, cfg::IMU_AXIS_MAP_Z, cfg::IMU_AXIS_SIGN_Z);
	const int32_t gx_raw =
		mapAxis(gyro_raw, cfg::IMU_AXIS_MAP_X, cfg::IMU_AXIS_SIGN_X);
	const int32_t gy_raw =
		mapAxis(gyro_raw, cfg::IMU_AXIS_MAP_Y, cfg::IMU_AXIS_SIGN_Y);
	const int32_t gz_raw =
		mapAxis(gyro_raw, cfg::IMU_AXIS_MAP_Z, cfg::IMU_AXIS_SIGN_Z);

	const float ax = (float)ax_raw * accel_scale;
	const float ay = (float)ay_raw * accel_scale;
	const float az = (float)az_raw * accel_scale;
	const float gx = (float)gx_raw * gyro_scale;
	const float gy = (float)gy_raw * gyro_scale;
	const float gz = (float)gz_raw * gyro_scale;
	const float a_norm = std::sqrt(ax * ax + ay * ay + az * az);
	const bool accel_norm_ok =
		std::fabs(a_norm - kG_mm_s2) <= cfg::IMU_ACCEL_NORM_MAX_DEV_MM_S2;

	uint32_t dt_ms = 0;
	if (_st.last_ms == 0) {
		_st.last_ms = now_ms;
	} else {
		dt_ms = now_ms - _st.last_ms;
		_st.last_ms = now_ms;
	}
	const float dt_s =
		(dt_ms > 0) ? std::max(0.0f, std::min(0.1f, dt_ms / 1000.0f)) : 0.0f;

	float a_long_lpf = ax;
	if (cfg::IMU_GRAVITY_TAU_MS > 0) {
		if (!_gravity_init) {
			_g_est_x = ax;
			_g_est_y = ay;
			_g_est_z = az;
			_gravity_init = true;
		}
		if (dt_s > 0.0f) {
			const float tau = (float)cfg::IMU_GRAVITY_TAU_MS / 1000.0f;
			const float alpha = (tau > 0.0f) ? (dt_s / (tau + dt_s)) : 1.0f;
			_g_est_x += alpha * (ax - _g_est_x);
			_g_est_y += alpha * (ay - _g_est_y);
			_g_est_z += alpha * (az - _g_est_z);
		}
		a_long_lpf = ax - _g_est_x;
	}

	float a_long_fusion = a_long_lpf;
	if (cfg::IMU_USE_FUSION) {
		if (!_fusion_init) {
			FusionAhrsInitialise(&_fusion);
			FusionAhrsSettings settings;
			settings.convention = FusionConventionNwu;
			settings.gain = cfg::IMU_FUSION_GAIN;
			settings.gyroscopeRange = gyroRangeDps_();
			settings.accelerationRejection = cfg::IMU_FUSION_ACC_REJ_DEG;
			settings.magneticRejection = 0.0f;
			settings.recoveryTriggerPeriod = cfg::IMU_FUSION_RECOVER_S;
			FusionAhrsSetSettings(&_fusion, settings);
			_fusion_init = true;
		}
		if (dt_s > 0.0f) {
			const FusionVector gyro = {gx, gy, gz};
			const FusionVector accel = {ax / kG_mm_s2, ay / kG_mm_s2,
										az / kG_mm_s2};
			FusionAhrsUpdateNoMagnetometer(&_fusion, gyro, accel, dt_s);
			const FusionVector lin = FusionAhrsGetLinearAcceleration(&_fusion);
			a_long_fusion = lin.axis.x * kG_mm_s2;
		}
	}

	_st.a_long_lpf_mm_s2 = a_long_lpf;
	_st.a_long_fusion_mm_s2 = a_long_fusion;
	_st.a_long_mm_s2 = cfg::IMU_USE_FUSION ? a_long_fusion : a_long_lpf;
	_st.gz_dps = gz;

	if (dt_ms == 0) {
		return;
	}

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
	if (braking && decel > 0.0f && accel_norm_ok) {
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
	const float accel_scale = accelScaleMmS2PerLsb_();
	const float gyro_scale = gyroScaleDpsPerLsb_();
	const float ax = (float)s.ax * accel_scale;
	const float ay = (float)s.ay * accel_scale;
	const float az = (float)s.az * accel_scale;
	const float gx = (float)s.gx * gyro_scale;
	const float gy = (float)s.gy * gyro_scale;
	const float gz = (float)s.gz * gyro_scale;
	const float a_norm = std::sqrt(ax * ax + ay * ay + az * az);
	const float g_dev = std::fabs(a_norm - kG_mm_s2);
	const float g_abs =
		std::max(std::fabs(gx), std::max(std::fabs(gy), std::fabs(gz)));
	if (g_abs > cfg::IMU_CALIB_GYRO_DPS ||
		g_dev > cfg::IMU_CALIB_ACCEL_DEV_MM_S2) {
		return;
	}

	if (_sum_n == 0)
		_calib_start_ms = now_ms;

	if ((now_ms - _calib_start_ms) > cfg::IMU_CALIBRATION_MS &&
		_sum_n >= cfg::IMU_CALIB_MIN_SAMPLES) {
		_bias_ax = (int32_t)(_sum_ax / (int64_t)_sum_n);
		_bias_ay = (int32_t)(_sum_ay / (int64_t)_sum_n);
		_bias_az = (int32_t)(_sum_az / (int64_t)_sum_n);
		_bias_gx = (int32_t)(_sum_gx / (int64_t)_sum_n);
		_bias_gy = (int32_t)(_sum_gy / (int64_t)_sum_n);
		_bias_gz = (int32_t)(_sum_gz / (int64_t)_sum_n);
		_st.calibrated = true;
		_gravity_init = false;
		return;
	}

	_sum_ax += s.ax;
	_sum_ay += s.ay;
	_sum_az += s.az;
	_sum_gx += s.gx;
	_sum_gy += s.gy;
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

float ImuEstimator::gyroRangeDps_() const {
	switch (cfg::IMU_GYRO_FS_SEL & 0x03) {
	case 0:
		return 250.0f;
	case 1:
		return 500.0f;
	case 2:
		return 1000.0f;
	case 3:
	default:
		return 2000.0f;
	}
}
