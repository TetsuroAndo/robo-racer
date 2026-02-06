#pragma once

#include <stdint.h>

struct DecelControlOutput {
	int16_t pwm_cmd = 0;      // <= 0 (brake)
	float a_tgt_mm_s2 = 0.0f; // negative
	float a_err_mm_s2 = 0.0f;
	float u_ff = 0.0f;
	float u_fb = 0.0f;
	float u_i = 0.0f;
	bool active = false;
};

class DecelController {
public:
	void reset();

	DecelControlOutput update(int16_t v_target_mm_s, float v_est_mm_s,
							  float a_long_lpf_mm_s2,
							  float a_brake_cap_mm_s2, float dt_s,
							  bool imu_calib, bool enable);

private:
	float _u_i = 0.0f;
};
