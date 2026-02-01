#pragma once

#include "../../src/config/Config.h"
#include "LidarScanData.hpp"
#include "sl_lidar.h"
#include "sl_lidar_driver.h"
#include <csignal>
#include <iostream>
#include <semaphore.h>
#include <unistd.h>
#include <cstdint>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <cmath>

#define UPDATED 1
#define NOUPDATED 0

struct	ShmLidarScanData {
	uint32_t seq;
	int32_t distance_mm[181];
};
