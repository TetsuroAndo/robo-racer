#pragma once

#include <cstddef>
#include <vector>

namespace planning {

struct Scan {
	float angle_min_deg = 0.0f;
	float angle_max_deg = 0.0f;
	float angle_step_deg = 1.0f;
	std::vector< int > ranges_mm;

	inline size_t bin_count() const {
		return ranges_mm.size();
	}

	inline float angleAt(size_t idx) const {
		return angle_min_deg + static_cast< float >(idx) * angle_step_deg;
	}
};

struct PlanOutput {
	int steer_deg = 0;
	int speed_hint_mm = 0;
	int target_distance_mm = 0;
	int front_min_distance_mm = 0;
	bool stop = false;
};

struct GapDescriptor {
	size_t start_idx = 0;
	size_t end_idx = 0;
	size_t best_idx = 0;
	int best_distance_mm = 0;
};

} // namespace planning

