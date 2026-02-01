#include "LidarScanData.hpp"

LidarScanData::LidarScanData() {
	for (int i = 0; i < 181; i++)
		distance_mm[i] = 0;
}

LidarScanData::~LidarScanData() {}

int32_t LidarScanData::getDistance(int32_t angle) const {
	if (angle < -90 || angle > 90)
		return (-1);

	int32_t index = angle + 90;
	return (distance_mm[index]);
}

bool LidarScanData::setDistance(int32_t angle, int32_t value) {
	if (angle < -90 || angle > 90)
		return (false);

	int32_t index = angle + 90;
	distance_mm[index] = value;
	return (true);
}

void LidarScanData::clear() {
	for (int i = 0; i < 181; i++)
		distance_mm[i] = 0;
}
