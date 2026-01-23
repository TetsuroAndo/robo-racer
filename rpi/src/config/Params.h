#pragma once

#include "config/Config.h"

namespace cfg {

struct Params {
	float fov_min_deg = -90.0f;
	float fov_max_deg = 90.0f;
	float angle_step_deg = 1.0f;

	int range_min_mm = 150;
	int range_max_mm = 6000;

	int free_threshold_mm = 700;
	int stop_distance_mm = 300;
	int front_sector_deg = 15;

	int vehicle_width_mm = 190;
	int safety_margin_mm = 30;
	int bubble_radius_mm = 250;
	int disparity_threshold_mm = 300;

	int smoothing_window = 1;
	int smoothing_mode = 1; // 0: off, 1: mean, 2: median

	float steer_slowdown_gain = 0.8f;
	float steer_slowdown_angle_deg = 30.0f;
	float steer_smooth_beta = 0.25f;
	int steer_rate_limit_deg = 10;

	float speed_gain = 1.0f;
	float accel_limit = 50.0f;

	int baseline_speed_div = 50;
	float baseline_min_angle_sign = -1.0f;

	int vehicle_half_width_mm() const {
		return (vehicle_width_mm / 2) + safety_margin_mm;
	}
	int max_speed_input() const {
		return cfg::SPEED_INPUT_LIMIT;
	}
};

} // namespace cfg
