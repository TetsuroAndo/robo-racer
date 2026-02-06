#pragma once

#include <cstdint>

struct Tsd20State {
	bool valid = false;
	bool ready = false;
	bool sensor_valid = false;
	int32_t mm = 0;
	int32_t fails = 0;
	int32_t period_ms = 0;
	uint64_t ts_us = 0;
};
