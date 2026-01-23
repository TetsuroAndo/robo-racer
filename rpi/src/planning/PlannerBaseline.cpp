#include "planning/PlannerBaseline.h"

#include <climits>
#include <cmath>

namespace planning {

PlanOutput PlannerBaseline::plan(const Scan &scan, const cfg::Params &params) {
	PlanOutput output;
	int max_distance = -1;
	// size_t max_idx = 0;
	int min_distance = INT_MAX;
	size_t min_idx = 0;

	for (size_t i = 0; i < scan.bin_count(); ++i) {
		int dist = scan.ranges_mm[i];
		if (dist <= 0 || dist >= params.range_max_mm)
			continue;
		if (dist > max_distance) {
			max_distance = dist;
			// max_idx = i;
		}
		if (dist < min_distance) {
			min_distance = dist;
			min_idx = i;
		}
	}

	if (max_distance <= 0) {
		output.stop = true;
		return output;
	}

	output.target_distance_mm = max_distance;
	output.speed_hint_mm = max_distance;
	output.front_min_distance_mm =
		(min_distance == INT_MAX) ? params.range_max_mm : min_distance;
	output.steer_deg = static_cast< int >(
		std::round(scan.angleAt(min_idx) * params.baseline_min_angle_sign));

	return output;
}

} // namespace planning
