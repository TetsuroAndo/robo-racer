#ifndef LIDARSCANDATA_HPP
#define LIDARSCANDATA_HPP

#include <cstdint>

class LidarScanData {
private:
	int32_t distance_mm[181];

public:
	LidarScanData();
	~LidarScanData();

	int32_t getDistance(int32_t angle) const;
	bool setDistance(int32_t angle, int32_t value);
	void clear();
};

#endif
