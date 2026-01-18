#include "Process.h"

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
		if (-70 <= i.angle && i.angle <= 70) {
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
