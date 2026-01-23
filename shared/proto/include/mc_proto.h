#pragma once

#include <stddef.h>
#include <stdint.h>

// This file is the Single Source of Truth for the MC protocol definitions.
// It is shared between the ESP32 firmware and the RPi controller.

#pragma pack(push, 1)

namespace mc::proto {

// Shared constants
static constexpr uint8_t MAGIC0 = 'M';
static constexpr uint8_t MAGIC1 = 'C';
static constexpr uint8_t VERSION = 1;

// Packet Types
// Consolidates enums from both firmware and RPi
enum class Type : uint8_t {
	DRIVE = 0x01,
	KILL = 0x02,
	MODE_SET = 0x03,
	PING = 0x04,

	LOG = 0x10,
	STATUS = 0x11,
	HILS_STATE = 0x12,
	ACK = 0x80,
};

// Packet Flags
enum Flags : uint8_t {
	FLAG_ACK_REQ = 1u << 0, // v1: only this bit is used
	FLAG_ACK = 1u << 1,		// reserved (v1 unused)
	FLAG_ERR = 1u << 2,		// reserved (v1 unused)
};

// Common packet header
struct Header {
	uint8_t magic[2];
	uint8_t ver;
	uint8_t type;
	uint8_t flags;
	uint16_t seq_le;
	uint16_t len_le;
};
static_assert(sizeof(Header) == 9, "Header must be 9 bytes on wire");

// --- Payloads ---

// CMD_DRIVE (RPi -> ESP32)
// As defined in firmware/src/comm/handlers/DriveHandler.cpp
struct DrivePayload {
	int16_t steer_cdeg;
	int16_t speed_mm_s;
	uint16_t ttl_ms_le;
	uint16_t dist_mm_le;
};
static_assert(sizeof(DrivePayload) == 8, "DrivePayload must be 8 bytes");

// CMD_MODE_SET (RPi -> ESP32)
struct ModeSetPayload {
	uint8_t mode; // 0=MANUAL, 1=AUTO
};
static_assert(sizeof(ModeSetPayload) == 1, "ModeSetPayload must be 1 byte");

// CMD_HILS_STATE (ESP32 -> RPi)
struct HilsStatePayload {
	uint32_t timestamp;
	int16_t throttle_raw;
	int16_t steer_cdeg;
	uint8_t flags;
};
static_assert(sizeof(HilsStatePayload) == 9,
			  "HilsStatePayload must be 9 bytes");

// CMD_STATUS (ESP32 -> RPi)
// As defined in firmware/src/main.cpp::StatusPayload
struct StatusPayload {
	uint8_t seq_applied;
	uint8_t auto_active;
	uint16_t faults_le;
	int16_t speed_mm_s_le;
	int16_t steer_cdeg_le;
	uint16_t age_ms_le;
};
static_assert(sizeof(StatusPayload) == 10, "StatusPayload must be 10 bytes");

// Other commands (KILL, PING, LOG, ACK) have no fixed payload struct.

} // namespace mc::proto

#pragma pack(pop)
