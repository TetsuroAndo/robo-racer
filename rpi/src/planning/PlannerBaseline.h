#pragma once

#include "planning/IPlanner.h"

namespace planning {

class PlannerBaseline : public IPlanner {
public:
	PlanOutput plan(const Scan &scan, const cfg::Params &params) override;
};

} // namespace planning
