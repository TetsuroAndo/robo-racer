#include "LidarReceiver.h"

#include "config/Config.h"

#include "sl_lidar.h"
#include "sl_lidar_driver.h"

#include <array>
#include <cerrno>
#include <cmath>
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
	return (static_cast< float >(n.angle_z_q14) *
			cfg::LIDAR_ANGLE_Q14_SCALE_DEG) /
		   cfg::LIDAR_ANGLE_Q14_DENOM;
}

unsigned int
dist_mm_floor_uint(const sl_lidar_response_measurement_node_hq_t &n) {
	return static_cast< unsigned int >(n.dist_mm_q2 >> 2);
}

float normalize(double angle) {
	double a =
		fmod(angle + cfg::LIDAR_ANGLE_WRAP_DEG, cfg::LIDAR_ANGLE_FULL_DEG);
	if (a < 0)
		a += cfg::LIDAR_ANGLE_FULL_DEG;
	return static_cast< float >(a - cfg::LIDAR_ANGLE_WRAP_DEG);
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
	sl_lidar_response_measurement_node_hq_t nodes[cfg::LIDAR_NODE_MAX];
	size_t nodeCount = sizeof(nodes) / sizeof(nodes[0]);

	// 1) まずデータ取得（nodeCount が “実データ数” に更新される）
	sl_result ans = _lidar->grabScanDataHq(nodes, nodeCount);
	if (!SL_IS_OK(ans) || nodeCount == 0) {
		return {};
	}

	// 2) 角度昇順に並べ替え（取得ではない）
	ans = _lidar->ascendScanData(nodes, nodeCount);
	if (!SL_IS_OK(ans)) {
		// 全点無効などで FAIL になることがあるので、ここで捨てる/続行は方針次第
		return {};
	}

	std::vector< LidarData > res;
	res.reserve(nodeCount);

	for (size_t i = 0; i < nodeCount; i++) {
		// 無効点（距離0）を弾くのが定番
		if (nodes[i].dist_mm_q2 == 0)
			continue;

		float angle = (static_cast< float >(nodes[i].angle_z_q14) *
					   cfg::LIDAR_ANGLE_Q14_SCALE_DEG) /
					  cfg::LIDAR_ANGLE_Q14_DENOM;
		angle = normalize(angle);

		unsigned int dist =
			static_cast< unsigned int >(nodes[i].dist_mm_q2 >> 2);
		if (dist < cfg::LIDAR_DIST_MIN_MM)
			continue;

		res.emplace_back(dist, angle);
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
