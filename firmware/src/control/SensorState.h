#pragma once

#include <stdint.h>

struct Tsd20State {
	bool ready = false;
	bool valid = false;
	uint16_t mm = 0;
	uint8_t fail_count = 0;
};
