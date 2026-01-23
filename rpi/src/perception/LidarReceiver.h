#pragma once

#include "perception/LidarTypes.h"
#include "sl_lidar.h"
#include "sl_lidar_driver.h"
#include <vector>

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
