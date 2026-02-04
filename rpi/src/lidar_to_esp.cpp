// lidar_to_esp.cpp

#include "lidar_to_esp.h"

#include "LidarReceiver.h"
#include "Process.h"
#include "Sender.h"
#include "config/Config.h"
#include "mc/core/Log.hpp"
#include "mc/core/Time.hpp"

#include <csignal>
#include <cstdint>
#include <iostream>
#include <sstream>

static volatile sig_atomic_t g_stop = 0;
static void on_sig(int) { g_stop = 1; }
static void show_cursor() { std::cout << "\x1b[?25h" << std::flush; }

static std::string make_run_id() {
	std::ostringstream oss;
	oss << std::hex << mc::core::Time::us();
	return oss.str();
}

int run_lidar_to_esp(const char *lidar_dev, int lidar_baud,
					 const char *esp_dev) {
	signal(SIGINT, on_sig);
	signal(SIGTERM, on_sig);
	std::atexit(show_cursor);
	mc::core::Logger::instance().setConsoleEnabled(false);

	LidarReceiver lidarReceiver(lidar_dev, lidar_baud);
	TelemetryEmitter telemetry;
	telemetry.setMetricsLogPath(cfg::DEFAULT_METRICSD_LOG);
	telemetry.setRateHz(cfg::TELEMETRY_DEFAULT_HZ);
	telemetry.setLevel(TelemetryLevel::Basic);
	Process process(&telemetry);
	Sender sender(esp_dev, &telemetry);
	const std::string run_id = make_run_id();
	uint64_t tick = 0;
	uint64_t scan_id = 0;
	float lastSteerAngle = 0.0f;

	while (!g_stop) {
		const auto &res = lidarReceiver.receive();
		const auto procResult =
			process.proc(res, lastSteerAngle, tick, scan_id, run_id);
		sender.send(procResult.speed, procResult.angle);
		lastSteerAngle = procResult.angle;
		++tick;
		++scan_id;
	}
	telemetry.shutdownUi();
	return 0;
}
