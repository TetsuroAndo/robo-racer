#pragma once
#include <cstdint>

namespace mc::core {

struct Time {
	static uint64_t us();
	static uint32_t ms();
};

} // namespace mc::core
