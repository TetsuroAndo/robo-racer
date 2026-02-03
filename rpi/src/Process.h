#pragma once

#include "LidarReceiver.h"
#include <cstdint>
#include <string>
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

	ProcResult proc(const std::vector< LidarData > &lidarData,
					float lastSteerAngle,
					uint64_t tick,
					uint64_t scan_id,
					const std::string &run_id) const;
};
