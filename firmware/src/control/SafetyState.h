#pragma once

#include "../config/Config.h"
#include <stdint.h>

struct SafetyState {
	bool kill_latched = false;
	bool auto_active = false;
	bool auto_inactive_seen = false;

	uint32_t last_hb_ms = 0;
	uint32_t hb_timeout_ms = cfg::HEARTBEAT_TIMEOUT_MS;

	uint16_t fault = 0;

	enum FaultBits : uint16_t {
		KILL_LATCHED = 1u << 0,
		HB_TIMEOUT = 1u << 1,
		TTL_EXPIRED = 1u << 2,
		AUTO_INACTIVE = 1u << 3,
	};

	void updateFaults() {
		fault = 0;
		if (kill_latched) {
			fault |= KILL_LATCHED;
		}
	}

	void markAutoInactive() { auto_inactive_seen = true; }

	bool consumeAutoInactive() {
		const bool seen = auto_inactive_seen;
		auto_inactive_seen = false;
		return seen;
	}
};
