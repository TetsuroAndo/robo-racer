#pragma once

#include "../../src/config/Config.h"
#include "LidarScanData.hpp"
#include "sl_lidar.h"
#include "sl_lidar_driver.h"
#include <csignal>
#include <semaphore.h>
#include <unistd.h>
#include <cstdint>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <cmath>

// LIDAR 用の更新フラグ。汎用的なマクロ名による他ヘッダとの衝突を避けるため、
// UPDATED / NOUPDATED ではなくプレフィックス付き名称を用いる。
#define LIDAR_UPDATED   1
#define LIDAR_NOUPDATED 0

struct	ShmLidarScanData {
	uint32_t seq;
	int32_t distance_mm[181];
};
