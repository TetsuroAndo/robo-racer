#include "perception/ScanPreprocessor.h"

#include <algorithm>
#include <cmath>

namespace {
constexpr int INF_DISTANCE = 100000;
} // namespace

namespace perception {

planning::Scan ScanPreprocessor::process(const std::vector< LidarData > &points,
										 const cfg::Params &params) const {
	float angle_span = params.fov_max_deg - params.fov_min_deg;
	if (angle_span < 0.0f) {
		angle_span = 0.0f;
	}
	const size_t bin_count = std::max< size_t >(
		1,
		static_cast< size_t >(std::floor(angle_span / params.angle_step_deg)) +
			1);

	planning::Scan scan;
	scan.angle_min_deg = params.fov_min_deg;
	scan.angle_max_deg =
		params.fov_min_deg +
		static_cast< float >(bin_count - 1) * params.angle_step_deg;
	scan.angle_step_deg = params.angle_step_deg;
	scan.ranges_mm.assign(bin_count, params.range_max_mm);

	const float step = params.angle_step_deg;
	for (const auto &point : points) {
		if (point.distance < params.range_min_mm ||
			point.distance > params.range_max_mm) {
			continue;
		}
		if (point.angle < scan.angle_min_deg ||
			point.angle > scan.angle_max_deg) {
			continue;
		}
		const float offset = (point.angle - scan.angle_min_deg) / step;
		const size_t idx = static_cast< size_t >(std::round(offset));
		if (idx >= bin_count)
			continue;
		scan.ranges_mm[idx] = std::min(scan.ranges_mm[idx], point.distance);
	}

	// replace untouched bins with INF marker
	for (size_t i = 0; i < bin_count; ++i) {
		if (scan.ranges_mm[i] >= params.range_max_mm) {
			scan.ranges_mm[i] = INF_DISTANCE;
		}
	}

	return scan;
}

} // namespace perception
