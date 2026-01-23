#pragma once

#include "config/Params.h"

namespace control {

class SteeringSmoother {
public:
	SteeringSmoother() = default;
	int update(int raw_angle_deg, const cfg::Params &params);

private:
	float last_output_deg_ = 0.0f;
	bool has_last_ = false;
};

} // namespace control
