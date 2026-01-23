#pragma once

#include "config/Params.h"
#include "planning/planning_types.h"

namespace control {

class SpeedPolicy {
public:
	SpeedPolicy() = default;
	int computeSpeedInput(const planning::PlanOutput &plan, int steer_deg,
					 const cfg::Params &params);

private:
	float last_speed_mm_s_ = 0.0f;
};

} // namespace control
