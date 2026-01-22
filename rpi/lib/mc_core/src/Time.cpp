#include "mc_core/Time.h"
#include <time.h>

namespace mc {

uint64_t Time::us() {
	timespec ts{};
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

uint32_t Time::ms() { return (uint32_t)(us() / 1000ULL); }

} // namespace mc
