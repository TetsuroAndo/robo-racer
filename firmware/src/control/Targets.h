#pragma once

#include "../config/Config.h"
#include <stdint.h>

struct Targets {
	int16_t speed_mm_s = 0;
	int16_t steer_cdeg = 0;
	uint16_t ttl_ms = cfg::DRIVE_TTL_DEFAULT_MS;
	uint16_t dist_mm = 0;
};
