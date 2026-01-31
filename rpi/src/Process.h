#pragma once

#include "LidarReceiver.h"
#include <vector>

struct ProcResult {
	int speed;
	int angle;

	ProcResult(int speed, int angle) : speed(speed), angle(angle) {}
};

class Process {
public:
	Process();
	~Process();

	ProcResult proc(const std::vector< LidarData > &lidarData, float lastSteerAngle = 0.0f) const;
};
