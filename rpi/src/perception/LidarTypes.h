#pragma once

struct LidarData {
	int distance;
	float angle;

	LidarData(int distance, float angle) : distance(distance), angle(angle) {}
};
