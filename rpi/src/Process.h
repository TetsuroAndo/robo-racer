#pragma once

#include "LidarReceiver.h"
#include "Telemetry.h"
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
	explicit Process(TelemetryEmitter *telemetry);
	~Process();

	ProcResult proc(const std::vector< LidarData > &lidarData,
					float lastSteerAngle,
					uint64_t tick,
					uint64_t scan_id,
					const std::string &run_id) const;

private:
	TelemetryEmitter *telemetry_;
};
