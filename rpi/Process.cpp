#include "Process.h"

#include <cmath>

Process::Process() {}

Process::~Process() {}

ProcResult Process::proc(const std::vector< LidarData > &lidarData) const {

	float max = 0;
	int maxDistance = -1;
	float min = 0;
	int minDistance = -1;
	for (const auto &i : lidarData) {
		if (-30 <= i.distance && i.distance <= 30) {
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

	return ProcResult(maxDistance / 50, min * -1);
}
