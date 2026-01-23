#include "doctest/doctest.h"

#include "config/Params.h"
#include "planning/PlannerFTG.h"
#include "planning/planning_types.h"

#include <cmath>
#include <vector>

namespace {

planning::Scan make_uniform_scan(size_t bins, float angle_min_deg,
								 float angle_step_deg, int default_dist) {
	planning::Scan scan;
	scan.angle_min_deg = angle_min_deg;
	scan.angle_step_deg = angle_step_deg;
	scan.angle_max_deg =
		angle_min_deg + static_cast< float >(bins - 1) * angle_step_deg;
	scan.ranges_mm.assign(bins, default_dist);
	return scan;
}

size_t angle_to_index(const planning::Scan &scan, float angle_deg) {
	float offset = (angle_deg - scan.angle_min_deg) / scan.angle_step_deg;
	return static_cast< size_t >(std::round(offset));
}

} // namespace

TEST_CASE("PlannerFTG stops when front minimum is below the stop threshold") {
	cfg::Params params;
	params.stop_distance_mm = 300;
	params.free_threshold_mm = 300;

	auto scan = make_uniform_scan(181, -90.0f, 1.0f, 1000);
	auto center = angle_to_index(scan, 0.0f);
	scan.ranges_mm[center] = 250; // front point below stop_distance_mm

	auto output = planning::PlannerFTG().plan(scan, params);
	CHECK(output.stop);
}

TEST_CASE("PlannerFTG yields a gap and does not stop when front is safe") {
	cfg::Params params;
	params.stop_distance_mm = 300;
	params.free_threshold_mm = 300;

	auto scan = make_uniform_scan(181, -90.0f, 1.0f, 1000);
	auto center = angle_to_index(scan, 0.0f);
	scan.ranges_mm[center] = 500; // above stop threshold but still free

	auto output = planning::PlannerFTG().plan(scan, params);
	CHECK(!output.stop);
	CHECK(output.front_min_distance_mm > params.stop_distance_mm);
	CHECK(output.speed_hint_mm > 0);
}
