// lidar_to_esp.cpp

#include "lidar_to_esp.h"
#include "uart.h"

#include "sl_lidar.h"
#include "sl_lidar_driver.h"

#include <array>
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <unistd.h>

static volatile sig_atomic_t g_stop = 0;
static void on_sig(int) { g_stop = 1; }

int run_lidar_to_esp() {
	while (!g_stop) {
		res = lidar->grabScanDataHq(nodes, nodeCount);
		if (SL_IS_FAIL(res)) {
			usleep(1000);
			continue;
		}

		lidar->ascendScanData(nodes, nodeCount);

		for (size_t i = 0; i < nodeCount; i++) {
			unsigned int deg = deg_floor_u(nodes[i]);
			if (deg >= 360)
				continue;
			dist_mm[deg] = dist_mm_floor_u(nodes[i]);
		}
	}

	// 停止処理
	std::cerr << "Stopping LIDAR..." << std::endl;
	lidar->stop();
	lidar->setMotorSpeed(0);
	lidar->disconnect();

	delete lidar;
	delete channel;
	close(esp_fd);
	return 0;
}
