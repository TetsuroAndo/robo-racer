#include "LidarReceiver.h"

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

namespace {
// 小数点切り捨て / unsigned int
float deg_float(const sl_lidar_response_measurement_node_hq_t &n) {
	return (static_cast< float >(n.angle_z_q14) * 90.0f) / (1U << 14);
}

unsigned int
dist_mm_floor_uint(const sl_lidar_response_measurement_node_hq_t &n) {
	return static_cast< unsigned int >(n.dist_mm_q2 >> 2);
}
} // namespace

LidarReceiver::LidarReceiver(const char *lidar_dev_c, int lidar_baud) {
	_init(lidar_dev_c, lidar_baud);
}

LidarReceiver::~LidarReceiver() {
	if (_lidar) {
		_lidar->stop();
		_lidar->setMotorSpeed(0);
		_lidar->disconnect();
	}

	delete _channel;
	delete _lidar;
}

std::vector< LidarData > LidarReceiver::receive() {
	std::array< unsigned int, 360 > dist_mm;
	sl_lidar_response_measurement_node_hq_t nodes[8192];
	dist_mm.fill(0);
	size_t nodeCount = sizeof(nodes) / sizeof(nodes[0]);
	_lidar->ascendScanData(nodes, nodeCount);

	std::vector< LidarData > res;
	for (size_t i = 0; i < nodeCount; i++) {
		float deg = deg_float(nodes[i]);
		if (deg < 0 || deg >= 360)
			continue;
		unsigned int dist = dist_mm_floor_uint(nodes[i]);
		if (dist < 5)
			continue;
		LidarData data(dist, deg);
		res.push_back(data);
	}
	return res;
}

void LidarReceiver::_init(const char *lidar_dev_c, int lidar_baud) {
	// --- Channel 作成 ---
	auto channelRes =
		sl::createSerialPortChannel(std::string(lidar_dev_c), lidar_baud);

	if (!channelRes) {
		std::fprintf(stderr, "Failed to create serial channel (%s, %d)\n",
					 lidar_dev_c, lidar_baud);
		exit(1);
	}
	_channel = channelRes.value;

	// --- Driver 作成 ---
	auto lidarRes = sl::createLidarDriver();
	if (!lidarRes) {
		std::fprintf(stderr, "Failed to create lidar driver\n");
		exit(1);
	}
	_lidar = lidarRes.value;

	// connect
	sl_result res = _lidar->connect(_channel);
	if (!SL_IS_OK(res)) {
		std::fprintf(stderr, "Failed to connect to LIDAR: %08x\n",
					 (unsigned int)res);
		exit(1);
	}

	// デバイス情報/ヘルス確認（SDKサンプルと同じ順序）
	sl_lidar_response_device_info_t devinfo;
	res = _lidar->getDeviceInfo(devinfo);
	if (!SL_IS_OK(res)) {
		std::fprintf(stderr, "getDeviceInfo failed: %08x\n", (unsigned int)res);
		exit(1);
	}

	sl_lidar_response_device_health_t health;
	res = _lidar->getHealth(health);
	if (!SL_IS_OK(res) || health.status == SL_LIDAR_STATUS_ERROR) {
		std::fprintf(stderr, "getHealth failed or unhealthy: %08x status=%u\n",
					 (unsigned int)res, (unsigned int)health.status);
		exit(1);
	}

	// モータ回転（サンプルと同じく開始前に回す）
	res = _lidar->setMotorSpeed();
	if (!SL_IS_OK(res)) {
		std::fprintf(stderr, "setMotorSpeed failed: %08x\n", (unsigned int)res);
		exit(1);
	}

	// スキャン開始
	sl::LidarScanMode scanMode;
	res = _lidar->startScan(false, true, 0, &scanMode);
	std::cerr << "LIDAR startScan result: " << res
			  << ", mode=" << scanMode.scan_mode << std::endl;
	if (!SL_IS_OK(res)) {
		std::fprintf(stderr, "startScan failed: %08x\n", (unsigned int)res);
		exit(1);
	}
}
