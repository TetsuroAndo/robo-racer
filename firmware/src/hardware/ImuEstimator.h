#pragma once

#include "Mpu6500.h"
#include <Fusion.h>
#include <stdint.h>

struct ImuEstimate {
	bool calibrated = false;
	float a_long_mm_s2 = 0.0f;
	float a_long_lpf_mm_s2 = 0.0f;
	float a_long_fusion_mm_s2 = 0.0f;
	float gz_dps = 0.0f;
	float v_est_mm_s = 0.0f;
	float a_brake_cap_mm_s2 = 0.0f;
	uint32_t last_ms = 0;
};

class ImuEstimator {
public:
	void reset(uint32_t now_ms);
	void update(const ImuSample &s,
				uint32_t now_ms,
				int16_t applied_speed_mm_s,
				int16_t target_speed_mm_s);
	const ImuEstimate &state() const { return _st; }

private:
	void updateBias_(const ImuSample &s, uint32_t now_ms);
	float accelScaleMmS2PerLsb_() const;
	float gyroScaleDpsPerLsb_() const;
	float gyroRangeDps_() const;

	ImuEstimate _st{};
	bool _calib_started = false;
	uint32_t _calib_start_ms = 0;
	int64_t _sum_ax = 0;
	int64_t _sum_ay = 0;
	int64_t _sum_az = 0;
	int64_t _sum_gx = 0;
	int64_t _sum_gy = 0;
	int64_t _sum_gz = 0;
	uint32_t _sum_n = 0;

	int32_t _bias_ax = 0;
	int32_t _bias_ay = 0;
	int32_t _bias_az = 0;
	int32_t _bias_gx = 0;
	int32_t _bias_gy = 0;
	int32_t _bias_gz = 0;

	uint32_t _zupt_ms = 0;
	float _g_est_x = 0.0f;
	float _g_est_y = 0.0f;
	float _g_est_z = 0.0f;
	bool _gravity_init = false;

	FusionAhrs _fusion{};
	bool _fusion_init = false;
};
