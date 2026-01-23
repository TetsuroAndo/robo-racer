#include "doctest/doctest.h"

#include "config/Params.h"
#include "control/SpeedPolicy.h"
#include "control/SteeringSmoother.h"
#include "perception/ScanPreprocessor.h"
#include "planning/PlannerFTG.h"
#include "planning/planning_types.h"

#include <vector>

TEST_CASE("Pipeline integration: stop propagates to speed") {
	cfg::Params params;
	params.stop_distance_mm = 300;
	params.free_threshold_mm = 300;
	params.smoothing_window = 1;

	std::vector< LidarData > points;
	points.emplace_back(250, 0.0f);
	points.emplace_back(1000, 30.0f);
	points.emplace_back(1000, -30.0f);

	perception::ScanPreprocessor preprocessor;
	auto scan = preprocessor.process(points, params);
	planning::PlannerFTG planner;
	auto plan = planner.plan(scan, params);
	control::SteeringSmoother smoother;
	int steer = smoother.update(plan.steer_deg, params);
	control::SpeedPolicy speed_policy;
	int speed = speed_policy.computeSpeedInput(plan, steer, params);

	CHECK(plan.stop);
	CHECK(speed == 0);
}

TEST_CASE("Pipeline integration: safe scan yields motion") {
	cfg::Params params;
	params.stop_distance_mm = 300;
	params.free_threshold_mm = 300;
	params.smoothing_window = 1;

	std::vector< LidarData > points;
	points.emplace_back(1200, -10.0f);
	points.emplace_back(1300, 0.0f);
	points.emplace_back(1100, 15.0f);

	perception::ScanPreprocessor preprocessor;
	auto scan = preprocessor.process(points, params);
	planning::PlannerFTG planner;
	auto plan = planner.plan(scan, params);
	control::SteeringSmoother smoother;
	int steer = smoother.update(plan.steer_deg, params);
	control::SpeedPolicy speed_policy;
	int speed = speed_policy.computeSpeedInput(plan, steer, params);

	CHECK(!plan.stop);
	CHECK(speed >= 0);
}

TEST_CASE("Pipeline integration: wider vehicle reduces speed") {
	cfg::Params narrow;
	narrow.stop_distance_mm = 300;
	narrow.free_threshold_mm = 300;
	narrow.vehicle_width_mm = 160;
	narrow.safety_margin_mm = 10;

	cfg::Params wide = narrow;
	wide.vehicle_width_mm = 260;
	wide.safety_margin_mm = 40;

	std::vector< LidarData > points;
	points.emplace_back(1200, -10.0f);
	points.emplace_back(1200, 0.0f);
	points.emplace_back(1200, 10.0f);

	perception::ScanPreprocessor preprocessor;
	auto scan_narrow = preprocessor.process(points, narrow);
	auto scan_wide = preprocessor.process(points, wide);

	planning::PlannerFTG planner;
	auto plan_narrow = planner.plan(scan_narrow, narrow);
	auto plan_wide = planner.plan(scan_wide, wide);

	control::SteeringSmoother smoother;
	control::SpeedPolicy policy;

	int steer_narrow = smoother.update(plan_narrow.steer_deg, narrow);
	int speed_narrow =
		policy.computeSpeedInput(plan_narrow, steer_narrow, narrow);

	control::SteeringSmoother smoother_wide;
	control::SpeedPolicy policy_wide;
	int steer_wide = smoother_wide.update(plan_wide.steer_deg, wide);
	int speed_wide = policy_wide.computeSpeedInput(plan_wide, steer_wide, wide);

	CHECK(speed_wide <= speed_narrow);
}
