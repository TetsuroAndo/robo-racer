#include "planning/PlannerFTG.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

namespace {
constexpr float kRadToDeg = 180.0f / std::numbers::pi_v< float >;
constexpr int kInfDistance = 100000;

} // namespace

namespace planning {

PlanOutput PlannerFTG::plan(const Scan &scan, const cfg::Params &params) {
	PlanOutput output;
	output.front_min_distance_mm = computeFrontMin(scan, params);
	if (output.front_min_distance_mm <= params.stop_distance_mm) {
		output.stop = true;
		return output;
	}

	auto inflated = inflateObstacles(scan, params);
	auto gaps = findGaps(scan, params, inflated);
	if (gaps.empty()) {
		output.stop = true;
		return output;
	}

	size_t best_idx = selectBestGap(gaps);
	const auto &chosen = gaps[best_idx];
	if (chosen.best_idx >= scan.bin_count()) {
		output.stop = true;
		return output;
	}

	output.target_distance_mm = scan.ranges_mm[chosen.best_idx];
	output.speed_hint_mm = output.target_distance_mm;
	output.steer_deg =
		static_cast< int >(std::round(scan.angleAt(chosen.best_idx)));
	output.stop = false;
	return output;
}

int PlannerFTG::computeFrontMin(const Scan &scan,
								const cfg::Params &params) const {
	if (scan.bin_count() == 0)
		return params.range_max_mm;
	const float front_min_angle =
		std::max(scan.angle_min_deg, -params.front_sector_deg * 1.0f);
	const float front_max_angle =
		std::min(scan.angle_max_deg, params.front_sector_deg * 1.0f);

	const auto toIndex = [&](float angle) {
		float offset = (angle - scan.angle_min_deg) / scan.angle_step_deg;
		if (offset < 0.0f)
			offset = 0.0f;
		const size_t candidate = static_cast< size_t >(std::round(offset));
		return std::min(candidate, scan.bin_count() - 1);
	};

	const size_t start = toIndex(front_min_angle);
	const size_t end = toIndex(front_max_angle);
	int best = params.range_max_mm;
	for (size_t i = start; i <= end && i < scan.bin_count(); ++i) {
		best = std::min(best, scan.ranges_mm[i]);
	}
	return best;
}

std::vector< int >
PlannerFTG::inflateObstacles(const Scan &scan,
							 const cfg::Params &params) const {
	std::vector< int > inflated = scan.ranges_mm;
	const float step = scan.angle_step_deg;
	if (inflated.empty() || step <= 0.0f)
		return inflated;

	const int half_width = params.vehicle_half_width_mm();
	for (size_t i = 0; i < inflated.size(); ++i) {
		int dist = inflated[i];
		if (dist <= 0 || dist >= kInfDistance)
			continue;

		const float delta_rad = std::atan2(static_cast< float >(half_width),
										   static_cast< float >(dist));
		const float delta_deg = delta_rad * kRadToDeg;
		const size_t delta_bins =
			static_cast< size_t >(std::ceil(delta_deg / step));
		const size_t lower = (i > delta_bins) ? i - delta_bins : 0;
		const size_t upper = std::min(inflated.size() - 1, i + delta_bins);
		for (size_t j = lower; j <= upper; ++j) {
			inflated[j] = std::min(inflated[j], dist);
		}
	}
	return inflated;
}

std::vector< GapDescriptor >
PlannerFTG::findGaps(const Scan &scan, const cfg::Params &params,
					 const std::vector< int > &inflated) const {
	std::vector< GapDescriptor > gaps;
	if (scan.bin_count() == 0)
		return gaps;

	bool in_gap = false;
	GapDescriptor current;

	for (size_t idx = 0; idx < inflated.size(); ++idx) {
		bool is_free = inflated[idx] >= params.free_threshold_mm;
		if (!is_free) {
			if (in_gap) {
				current.end_idx = idx - 1;
				if (current.best_idx >= current.start_idx)
					gaps.push_back(current);
				in_gap = false;
			}
			continue;
		}
		if (!in_gap) {
			in_gap = true;
			current = GapDescriptor{};
			current.start_idx = idx;
			current.best_idx = idx;
			current.best_distance_mm = 0;
		}
		current.end_idx = idx;
		int raw = scan.ranges_mm[idx];
		int effective = std::min(raw, params.range_max_mm);
		if (effective > current.best_distance_mm) {
			current.best_distance_mm = effective;
			current.best_idx = idx;
		}
	}

	if (in_gap) {
		current.end_idx = inflated.size() - 1;
		if (current.best_idx >= current.start_idx)
			gaps.push_back(current);
	}
	return gaps;
}

size_t
PlannerFTG::selectBestGap(const std::vector< GapDescriptor > &gaps) const {
	if (gaps.empty())
		return 0;
	float best_score = -1.0f;
	size_t best_idx = 0;
	for (size_t i = 0; i < gaps.size(); ++i) {
		const auto &gap = gaps[i];
		const float span =
			static_cast< float >(gap.end_idx - gap.start_idx + 1);
		const float score = span + gap.best_distance_mm / 1000.0f;
		if (score > best_score) {
			best_score = score;
			best_idx = i;
		}
	}
	return best_idx;
}

} // namespace planning
