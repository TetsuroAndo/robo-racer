#include "app/lidar_to_esp.h"

#include "comm/Sender.h"
#include "config/Config.h"
#include "config/Params.h"
#include "control/SpeedPolicy.h"
#include "control/SteeringSmoother.h"
#include "perception/LidarReceiver.h"
#include "perception/ScanPreprocessor.h"
#include "planning/IPlanner.h"
#include "planning/PlannerBaseline.h"
#include "planning/PlannerFTG.h"

#include <algorithm>
#include <cctype>
#include <csignal>
#include <memory>
#include <string>

static volatile sig_atomic_t g_stop = 0;
static void on_sig(int) { g_stop = 1; }

namespace {
std::unique_ptr< planning::IPlanner > createPlanner(const std::string &name) {
	auto lower = name;
	std::transform(
		lower.begin(), lower.end(), lower.begin(),
		[](unsigned char c) { return static_cast< char >(std::tolower(c)); });
	if (lower == "baseline") {
		return std::make_unique< planning::PlannerBaseline >();
	}
	return std::make_unique< planning::PlannerFTG >();
}
} // namespace

int run_lidar_to_esp(const char *lidar_dev, int lidar_baud, const char *esp_dev,
					 const char *planner_name) {
	signal(SIGINT, on_sig);
	signal(SIGTERM, on_sig);

	cfg::Params params;
	perception::LidarReceiver receiver(lidar_dev, lidar_baud);
	perception::ScanPreprocessor preprocessor;
	control::SteeringSmoother smoother;
	control::SpeedPolicy speed_policy;
	auto planner = createPlanner(planner_name ? planner_name : "ftg");
	comm::Sender sender(esp_dev);

	while (!g_stop) {
		auto raw = receiver.receive();
		auto scan = preprocessor.process(raw, params);
		auto output = planner->plan(scan, params);
		int steer = smoother.update(output.steer_deg, params);
		int speed = speed_policy.computeSpeedInput(output, steer, params);
		sender.send(speed, steer);
	}

	return 0;
}
