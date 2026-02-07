#pragma once

#include "SensorState.h"
#include "StopLevel.h"
#include "Targets.h"
#include "../comm/registry.h"
#include "../hardware/ImuEstimator.h"
#include <stdint.h>

struct Tsd20Diag {
	float d_allow = 0.0f;
	float margin_eff = 0.0f;
	float margin_pred = 0.0f;
	float steer_ratio = 0.0f;
	float a_cap = 0.0f;
	float d_travel = 0.0f;
	float v_est = 0.0f;
	float a_long = 0.0f;
	float age_ms = 0.0f;
	float tau = 0.0f;
	float v_max = 0.0f;
	float v_cap = 0.0f;
	bool clamped = false;
	uint8_t reason = 0;
	/// TSD20 STOP/MARGIN/AGE_STALE で速度0にされたとき true。BrakeController 用。
	bool stop_requested = false;
	/// stop_requested 時の理由。BrakeController で duty 段階化に使用。
	StopLevel stop_level = StopLevel::NONE;
};

class Tsd20Limiter {
public:
	enum Reason : uint8_t {
		TSD_DISABLED = 0,
		TSD_NONPOSITIVE = 1,
		TSD_MANUAL_SKIP = 2,
		TSD_NOT_READY = 3,
		TSD_INVALID = 4,
		TSD_MARGIN = 5,
		TSD_CLAMP = 6,
		TSD_OK = 7,
		TSD_STOP = 8,
		TSD_AGE_STALE = 9,
	};

	int16_t limit(int16_t speed_mm_s, mc::Mode mode,
				 const Tsd20State &tsd, const ImuEstimate &imu,
				 bool imu_valid, int16_t steer_cdeg, Tsd20Diag *diag) const;
};
