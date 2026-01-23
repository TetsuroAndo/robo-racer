#include "doctest/doctest.h"

#include "config/Params.h"
#include "control/SpeedPolicy.h"
#include "planning/planning_types.h"

TEST_CASE("SpeedPolicy returns zero when plan requests stop") {
	cfg::Params params;
	control::SpeedPolicy policy;
	planning::PlanOutput plan;
	plan.stop = true;

	int speed = policy.computeSpeedInput(plan, 0, params);
	CHECK(speed == 0);
}

TEST_CASE("SpeedPolicy respects front distance limit") {
	cfg::Params params;
	params.stop_distance_mm = 300;
	params.accel_limit = 1000.0f;

	planning::PlanOutput plan;
	plan.stop = false;
	plan.front_min_distance_mm = 320;
	plan.target_distance_mm = 1000;

	control::SpeedPolicy slow_policy;
	int slow = slow_policy.computeSpeedInput(plan, 0, params);

	plan.front_min_distance_mm = 500;
	control::SpeedPolicy fast_policy;
	int fast = fast_policy.computeSpeedInput(plan, 0, params);

	CHECK(fast >= slow);
}

TEST_CASE("SpeedPolicy slows down when steer increases") {
	cfg::Params params;
	params.stop_distance_mm = 300;

	planning::PlanOutput plan;
	plan.stop = false;
	plan.front_min_distance_mm = 500;
	plan.target_distance_mm = 1200;

	control::SpeedPolicy straight;
	int straight_speed = straight.computeSpeedInput(plan, 0, params);

	control::SpeedPolicy turning;
	int turning_speed = turning.computeSpeedInput(plan, 30, params);

	CHECK(turning_speed <= straight_speed);
}
