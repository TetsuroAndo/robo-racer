#pragma once

#include <stddef.h>
#include <stdint.h>

// clang-format off
namespace proto {

static constexpr uint8_t VER = 1;
static constexpr size_t MAX_PAYLOAD = 64;


enum class Type : uint8_t {
	AUTO_SETPOINT = 0x01,
	AUTO_MODE     = 0x02,
	HEARTBEAT     = 0x03,
	KILL          = 0x04,
	CLEAR_KILL    = 0x05,
	ACK           = 0x81,
	STATUS        = 0x82,
};

enum : uint8_t {
	FLAG_ACK_REQ = 1u << 0,
};

#pragma pack(push, 1)
struct Header {
	uint8_t  ver;
	uint8_t  type;
	uint8_t  flags;
	uint8_t  seq;
	uint16_t len;
};

struct AutoModePayload {
	uint8_t enable;
	uint8_t reason;
};

struct AutoSetpointPayload {
	int16_t  steer_cdeg;
	int16_t  speed_cmd;
	uint16_t ttl_ms;
	uint16_t distance_mm;
};

struct AckPayload {
	uint8_t type_echo;
	uint8_t seq_echo;
	uint8_t code;
	uint8_t detail;
};

struct StatusPayload {
	uint8_t  seq_applied;
	uint8_t  auto_active;
	uint16_t fault;
	int16_t  speed_now;
	int16_t  steer_now_cdeg;
	uint16_t age_ms;
};
#pragma pack(pop)

static constexpr size_t HEADER_SIZE = sizeof(Header);
static constexpr size_t CRC_SIZE = 2;
static constexpr size_t MAX_FRAME = HEADER_SIZE + MAX_PAYLOAD + CRC_SIZE;
static constexpr size_t MAX_ENCODED =
	MAX_FRAME + (MAX_FRAME / 254) + 1;

} // namespace proto
// clang-format on
