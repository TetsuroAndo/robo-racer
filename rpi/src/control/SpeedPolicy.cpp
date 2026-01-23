#include "control/SpeedPolicy.h"

#include <algorithm>
#include <cmath>

namespace control {

namespace {
constexpr float kSteerCap = 1.0f;
}

int SpeedPolicy::computeSpeedInput(const planning::PlanOutput &plan,
								   int steer_deg, const cfg::Params &params) {
	if (plan.stop) {
		last_speed_mm_s_ = 0.0f;
		return 0;
	}

	float front_gap = static_cast< float >(plan.front_min_distance_mm -
										   params.stop_distance_mm);
	front_gap = std::max(front_gap, 0.0f);
	float v_front = front_gap * params.speed_gain;
	float v_target =
		static_cast< float >(plan.target_distance_mm) * params.speed_gain;
	float v_cmd = std::min(v_front, v_target);

	float steer_norm = 0.0f;
	if (params.steer_slowdown_angle_deg > 0.0f) {
		steer_norm =
			std::min(kSteerCap, std::abs(static_cast< float >(steer_deg)) /
									params.steer_slowdown_angle_deg);
	}
	float steer_factor = 1.0f - params.steer_slowdown_gain * steer_norm;
	steer_factor = std::clamp(steer_factor, 0.0f, 1.0f);
	v_cmd *= steer_factor;

	v_cmd = std::min(v_cmd, static_cast< float >(cfg::SPEED_MM_S_MAX));

	float delta = v_cmd - last_speed_mm_s_;
	if (params.accel_limit > 0.0f) {
		delta = std::clamp(delta, -params.accel_limit, params.accel_limit);
	}
	v_cmd = last_speed_mm_s_ + delta;
	last_speed_mm_s_ = v_cmd;

	if (cfg::SPEED_MM_S_MAX <= 0)
		return 0;
	int speed_input = static_cast< int >(
		std::lround((v_cmd / cfg::SPEED_MM_S_MAX) * cfg::SPEED_INPUT_LIMIT));
	return std::clamp(speed_input, -cfg::SPEED_INPUT_LIMIT,
					  cfg::SPEED_INPUT_LIMIT);
}

} // namespace control
