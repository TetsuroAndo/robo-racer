#include "control/SteeringSmoother.h"

#include <algorithm>
#include <cmath>

namespace control {

int SteeringSmoother::update(int raw_angle_deg, const cfg::Params &params) {
	float target = static_cast< float >(raw_angle_deg);
	if (!has_last_) {
		last_output_deg_ = target;
		has_last_ = true;
	}

	const float beta = std::clamp(params.steer_smooth_beta, 0.0f, 1.0f);
	float smoothed = last_output_deg_ + beta * (target - last_output_deg_);
	float max_delta =
		static_cast< float >(std::abs(params.steer_rate_limit_deg));
	float delta = smoothed - last_output_deg_;
	delta = std::clamp(delta, -max_delta, max_delta);
	last_output_deg_ += delta;

	return static_cast< int >(std::lround(last_output_deg_));
}

} // namespace control
