#pragma once

#include <cstdint>

namespace mc::proto {

#pragma pack(push, 1)

struct DrivePayload {
	int16_t steer_cdeg;
	int16_t speed_mm_s;
	uint16_t ttl_ms_le;
	uint16_t distance_mm_le;
};

struct ModeSetPayload {
	uint8_t mode;
	uint8_t reason;
};

struct KillPayload {
	uint8_t reason;
	uint8_t reserved;
};

struct PingPayload {
	uint32_t nonce_le;
};

struct LogPayload {
	uint8_t level;
	uint8_t category;
	uint16_t code_le;
	uint32_t t_ms_le;
	int32_t a0_le;
	int32_t a1_le;
	int32_t a2_le;
	int32_t a3_le;
};

struct StatusPayload {
	uint8_t seq_applied;
	uint8_t auto_active;
	uint16_t faults_le;
	int16_t speed_mm_s_le;
	int16_t steer_cdeg_le;
	uint16_t age_ms_le;
};

struct AckPayload {
	uint16_t seq_ack_le;
	uint8_t type;
	uint8_t status;
};

#pragma pack(pop)

} // namespace mc::proto
