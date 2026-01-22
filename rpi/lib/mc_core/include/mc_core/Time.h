#pragma once
#include <cstdint>

namespace mc {

struct Time {
	static uint64_t us();
	static uint32_t ms();
};

} // namespace mc
