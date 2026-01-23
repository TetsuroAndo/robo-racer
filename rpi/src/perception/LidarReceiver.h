#pragma once

#include "sl_lidar.h"
#include "sl_lidar_driver.h"
#include <vector>

struct LidarData {
	int distance;
	float angle;

	LidarData(int distance, float angle) : distance(distance), angle(angle) {}
};

namespace perception {

class LidarReceiver {
public:
	LidarReceiver(const char *lidar_dev_c, int lidar_baud);
	~LidarReceiver();

	std::vector< LidarData > receive();

private:
	void _init(const char *lidar_dev_c, int lidar_baud);
	sl::IChannel *_channel;
	sl::ILidarDriver *_lidar;
};

} // namespace perception
