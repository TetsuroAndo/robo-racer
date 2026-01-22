#include "Process.h"
#include "config/Config.h"

#include <cmath>
#include <iostream>

Process::Process() {}

Process::~Process() {}

ProcResult Process::proc(const std::vector< LidarData > &lidarData) const {
	float max = 0;
	int maxDistance = -1;
	float min = 0;
	int minDistance = INT32_MAX;
	// std::cout << lidarData.size() << "\n";
	for (const auto &i : lidarData) {
		if (cfg::PROCESS_ANGLE_MIN_DEG <= i.angle &&
			i.angle <= cfg::PROCESS_ANGLE_MAX_DEG) {
			if (maxDistance < i.distance) {
				max = i.angle;
				maxDistance = i.distance;
			}
			if (i.distance < minDistance) {
				min = i.angle;
				minDistance = i.distance;
			}
		}
	}
	return ProcResult(maxDistance / cfg::PROCESS_SPEED_DIV,
					  min * cfg::PROCESS_MIN_ANGLE_SIGN);
}
