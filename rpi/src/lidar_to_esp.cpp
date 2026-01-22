// lidar_to_esp.cpp

#include "lidar_to_esp.h"

#include "LidarReceiver.h"
#include "Process.h"
#include "Sender.h"

#include <csignal>

static volatile sig_atomic_t g_stop = 0;
static void on_sig(int) { g_stop = 1; }

int run_lidar_to_esp(const char *lidar_dev, int lidar_baud,
					 const char *esp_dev) {
	signal(SIGINT, on_sig);
	signal(SIGTERM, on_sig);

	LidarReceiver lidarReceiver(lidar_dev, lidar_baud);
	Process process;
	Sender sender(esp_dev);

	while (!g_stop) {
		const auto &res = lidarReceiver.receive();
		const auto procResult = process.proc(res);
		sender.send(procResult.speed, procResult.angle);
	}
	return 0;
}
