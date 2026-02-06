#pragma once

#include "AbsController.h"
#include "Tsd20Limiter.h"
#include "Targets.h"
#include "../comm/registry.h"
#include "../hardware/ImuEstimator.h"
#include <stdint.h>

/// BrakeController 用：stop_requested の理由（duty 段階化に使用）
enum class StopLevel : uint8_t {
	NONE = 0,
	STOP = 1,   // 壁寸前・最大ブレーキ
	MARGIN = 2, // マージン内・中程度
	STALE = 3,  // 計測古い・最小（推力カット優先）
};

struct SafetyDiag {
	Tsd20Diag tsd{};
	AbsDiag abs{};
};

struct SafetyResult {
	Targets targets{};
	bool brake_mode = false;
	/// TSD20 STOP/MARGIN/AGE_STALE で速度0にされたとき true。BrakeController 用。
	bool stop_requested = false;
	/// stop_requested 時の理由。BrakeController で duty 段階化に使用。
	StopLevel stop_level = StopLevel::NONE;
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
