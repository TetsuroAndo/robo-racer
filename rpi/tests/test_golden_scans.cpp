#include "doctest/doctest.h"

#include "config/Params.h"
#include "planning/PlannerFTG.h"
#include "planning/planning_types.h"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {

struct GoldenCase {
	planning::Scan scan;
	bool expected_stop = false;
	int expected_steer_deg = 0;
	int expected_speed_hint = 0;
};

GoldenCase load_golden(const std::string &path) {
	GoldenCase result;
	std::ifstream in(path);
	CHECK(in.good());

	std::string line;
	float angle_min = -90.0f;
	float angle_step = 1.0f;
	while (std::getline(in, line)) {
		if (line.empty())
			continue;
		if (line.rfind("#", 0) == 0) {
			if (line.find("angle_min=") != std::string::npos) {
				angle_min = std::stof(line.substr(line.find('=') + 1));
			} else if (line.find("angle_step=") != std::string::npos) {
				angle_step = std::stof(line.substr(line.find('=') + 1));
			} else if (line.find("expected_stop=") != std::string::npos) {
				result.expected_stop = (line.find("true") != std::string::npos);
			} else if (line.find("expected_steer=") != std::string::npos) {
				result.expected_steer_deg =
					std::stoi(line.substr(line.find('=') + 1));
			} else if (line.find("expected_speed_hint=") != std::string::npos) {
				result.expected_speed_hint =
					std::stoi(line.substr(line.find('=') + 1));
			}
			continue;
		}
		result.scan.ranges_mm.push_back(std::stoi(line));
	}

	result.scan.angle_min_deg = angle_min;
	result.scan.angle_step_deg = angle_step;
	result.scan.angle_max_deg =
		angle_min +
		static_cast< float >(result.scan.ranges_mm.size() - 1) * angle_step;
	return result;
}

} // namespace

static void assert_golden(const char *path) {
	cfg::Params params;
	params.stop_distance_mm = 300;
	params.free_threshold_mm = 300;

	auto golden = load_golden(path);
	auto plan = planning::PlannerFTG().plan(golden.scan, params);

	CHECK(plan.stop == golden.expected_stop);
	if (!golden.expected_stop) {
		CHECK(plan.speed_hint_mm >= golden.expected_speed_hint);
	}
	if (golden.expected_steer_deg != 0) {
		if (golden.expected_steer_deg < 0) {
			CHECK(plan.steer_deg <= golden.expected_steer_deg);
		} else {
			CHECK(plan.steer_deg >= golden.expected_steer_deg);
		}
	}
}

TEST_CASE("Golden scan: open space") {
	assert_golden("rpi/tests/fixtures/scans/open_space.txt");
}

TEST_CASE("Golden scan: left turn") {
	assert_golden("rpi/tests/fixtures/scans/left_turn.txt");
}

TEST_CASE("Golden scan: corridor") {
	assert_golden("rpi/tests/fixtures/scans/corridor.txt");
}

TEST_CASE("Golden scan: chicane") {
	assert_golden("rpi/tests/fixtures/scans/chicane.txt");
}
