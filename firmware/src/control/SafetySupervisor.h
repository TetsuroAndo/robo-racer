#pragma once

#include "AbsController.h"
#include "Tsd20Limiter.h"
#include "Targets.h"
#include "../comm/registry.h"
#include "../hardware/ImuEstimator.h"
#include <stdint.h>

struct SafetyDiag {
	Tsd20Diag tsd{};
	AbsDiag abs{};
};

struct SafetyResult {
	Targets targets{};
	bool brake_mode = false;
};

class SafetySupervisor {
public:
	SafetyResult apply(uint32_t now_ms, float dt_s, const Targets &desired,
				  mc::Mode mode, const Tsd20State &tsd,
				  const ImuEstimate &imu, bool imu_valid,
				  bool abs_allowed, SafetyDiag *diag);

private:
	Tsd20Limiter _tsd;
	AbsController _abs;
};
