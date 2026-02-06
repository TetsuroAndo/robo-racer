#pragma once

#include "SensorState.h"
#include "../hardware/ImuEstimator.h"
#include <stdint.h>

struct AbsDiag {
	uint8_t reason = 0;
	bool active = false;
	float duty = 0.0f;
	float v_cmd = 0.0f;
	float v_est = 0.0f;
	float a_target = 0.0f;
	float a_cap = 0.0f;
	float decel = 0.0f;
	float dt_s = 0.0f;
};

class AbsController {
public:
	enum Reason : uint8_t {
		ABS_DISABLED = 0,
		ABS_NO_IMU = 1,
		ABS_NOT_CALIB = 2,
		ABS_REVERSE = 3,
		ABS_INACTIVE = 4,
		ABS_ACTIVE = 5,
	};

	void reset(uint32_t now_ms);

	int16_t apply(uint32_t now_ms, float dt_s, int16_t speed_mm_s,
				 bool allow_abs, const ImuEstimate &imu, bool imu_valid,
				 const Tsd20State &tsd, AbsDiag *diag, bool *active_out);

private:
	bool _active = false;
	uint32_t _cycle_ms = 0;
	float _duty = 0.0f;
	float _i = 0.0f;
	uint32_t _hold_until_ms = 0;
};
