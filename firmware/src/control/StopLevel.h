#pragma once
#include <stdint.h>

/// BrakeController 用：stop_requested の理由（duty 段階化に使用）
enum class StopLevel : uint8_t {
	NONE = 0,
	STOP = 1,   // 壁寸前・最大ブレーキ
	MARGIN = 2, // マージン内・中程度
	STALE = 3,  // 計測古い・最小（推力カット優先）
};
