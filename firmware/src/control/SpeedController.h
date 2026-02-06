#pragma once

#include <stdint.h>

struct SpeedControlOutput {
	int16_t pwm_cmd = 0;
	int16_t pwm_ff = 0;
	float error_mm_s = 0.0f;
	float integrator = 0.0f;
	bool saturated = false;
	bool active = false;
	bool calibrated = false;
};

class SpeedController {
public:
	void reset();

	SpeedControlOutput update(int16_t v_target_mm_s, float v_est_mm_s,
							  float dt_s, bool imu_calibrated, bool active,
							  bool brake_mode);

private:
	float _i = 0.0f;
	int16_t _last_target_mm_s = 0;

	int16_t speedToPwm_(int16_t v_target_mm_s) const;
};
