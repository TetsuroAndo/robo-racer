#pragma once

#include "config/Params.h"
#include "planning/planning_types.h"

namespace planning {

class IPlanner {
public:
	virtual ~IPlanner() = default;
	virtual PlanOutput plan(const Scan &scan, const cfg::Params &params) = 0;
};

} // namespace planning
