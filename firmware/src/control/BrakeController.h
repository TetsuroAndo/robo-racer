#pragma once

#include "StopLevel.h"
#include <stdint.h>

struct BrakeControllerOutput {
	uint8_t brake_duty = 0; // 0 または BRAKE_PWM_MIN..BRAKE_PWM_MAX（アクティブブレーキ用）
	bool active = false;
};

class BrakeController {
public:
	BrakeControllerOutput update(bool stop_requested, StopLevel stop_level,
								 uint16_t d_mm, float v_est_mm_s,
								 bool v_est_valid, uint32_t now_ms);

private:
	uint32_t _stop_since_ms = 0;
	uint32_t _last_stop_ms = 0;
	float _brake_ramp = 0.0f;
	StopLevel _latched_level = StopLevel::NONE;
};
