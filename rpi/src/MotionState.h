#pragma once

#include <cstdint>

struct MotionState {
	bool valid = false;
	bool calibrated = false;
	bool brake_mode = false;
	int16_t a_long_mm_s2 = 0;
	int16_t v_est_mm_s = 0;
	uint16_t a_brake_cap_mm_s2 = 0;
	float yaw_dps = 0.0f;
	uint16_t age_ms = 0;
	uint64_t ts_us = 0;
};
