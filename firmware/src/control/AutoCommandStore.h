#pragma once

#include "../comm/protocol/Protocol.h"
#include "../config/Config.h"
#include <stdint.h>

struct AutoCommandStore {
	bool has = false;
	uint8_t seq = 0;
	uint8_t applied_seq = 0;
	proto::AutoSetpointPayload sp{};
	uint32_t rx_ms = 0;

	void set(uint8_t s, const proto::AutoSetpointPayload &p, uint32_t now_ms) {
		has = true;
		seq = s;
		sp = p;
		rx_ms = now_ms;
	}

	bool isNewerSeq(uint8_t s) const {
		if (!has) {
			return true;
		}
		const uint8_t delta = static_cast<uint8_t>(s - seq);
		return delta != 0 && delta < cfg::AUTO_CMD_SEQ_WINDOW;
	}

	bool ttlExpired(uint32_t now_ms) const {
		if (!has) {
			return true;
		}
		return (uint32_t)(now_ms - rx_ms) > (uint32_t)sp.ttl_ms;
	}

	uint16_t ageMs(uint32_t now_ms) const {
		if (!has) {
			return cfg::AUTO_CMD_AGE_UNKNOWN_MS;
		}
		return (uint16_t)(now_ms - rx_ms);
	}
};
