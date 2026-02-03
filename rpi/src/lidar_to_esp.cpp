// lidar_to_esp.cpp

#include "lidar_to_esp.h"

#include "LidarReceiver.h"
#include "Process.h"
#include "Sender.h"
#include "mc/core/Time.hpp"

#include <csignal>
#include <cstdint>
#include <sstream>

static volatile sig_atomic_t g_stop = 0;
static void on_sig(int) { g_stop = 1; }

static std::string make_run_id() {
	std::ostringstream oss;
	oss << std::hex << mc::core::Time::us();
	return oss.str();
}

int run_lidar_to_esp(const char *lidar_dev, int lidar_baud,
					 const char *esp_dev) {
	signal(SIGINT, on_sig);
	signal(SIGTERM, on_sig);

	LidarReceiver lidarReceiver(lidar_dev, lidar_baud);
	Process process;
	Sender sender(esp_dev);
	const std::string run_id = make_run_id();
	uint64_t tick = 0;
	float lastSteerAngle = 0.0f;

	while (!g_stop) {
		const auto &res = lidarReceiver.receive();
		const auto procResult =
			process.proc(res, lastSteerAngle, tick, tick, run_id);
		sender.send(procResult.speed, procResult.angle);
		lastSteerAngle = procResult.angle;
		++tick;
	}
	return 0;
}
