#pragma once

#include "LidarReceiver.h"
#include <vector>

struct ProcResult {
	int speed;
	int angle;

	ProcResult(int speed, int angle) : speed(speed), angle(angle) {}
};

class Process {
	Process();
	~Process();

	ProcResult proc(std::vector< LidarData > &lidarData);
};
