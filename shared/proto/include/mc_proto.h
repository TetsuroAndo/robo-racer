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
	IMU_STATUS = 0x13,

	// IPC topics (RPi internal)
	IPC_LIDAR_SCAN = 0x20,
	IPC_LIDAR_SUMMARY = 0x21,
	IPC_IMU_SAMPLE = 0x22,
	IPC_DRIVE_CMD = 0x23,
	IPC_VEHICLE_STATUS = 0x24,
	IPC_METRICS = 0x25,
	IPC_LOG_RECORD = 0x26,
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

// IMU_STATUS (ESP32 -> RPi)
// As defined in firmware/src/main.cpp::ImuStatusPayload
struct ImuStatusPayload {
	int16_t a_long_mm_s2_le;
	int16_t v_est_mm_s_le;
	uint16_t a_brake_cap_mm_s2_le;
	int16_t yaw_dps_x10_le;
	uint16_t age_ms_le;
	uint8_t flags;
	uint8_t reserved;
};
static_assert(sizeof(ImuStatusPayload) == 12, "ImuStatusPayload must be 12 bytes");

// IPC_LIDAR_SCAN (RPi internal)
// Chunked scan payload header (data follows as uint16 mm array).
struct LidarScanChunkPayload {
	uint32_t ts_ms;
	uint16_t scan_id;
	int16_t angle_start_cdeg;
	int16_t angle_step_cdeg;
	uint8_t chunk_index;
	uint8_t chunk_count;
	uint8_t point_count;
	uint8_t encoding; // 0=uint16 mm
};
static_assert(sizeof(LidarScanChunkPayload) == 14,
			  "LidarScanChunkPayload must be 14 bytes");

// IPC_LIDAR_SUMMARY (RPi internal)
struct LidarSummaryPayload {
	uint32_t ts_ms;
	int16_t best_heading_cdeg;
	uint16_t best_distance_mm;
	uint16_t min_distance_mm;
	int16_t min_distance_heading_cdeg;
	uint8_t confidence;
	uint8_t flags;
};
static_assert(sizeof(LidarSummaryPayload) == 14,
			  "LidarSummaryPayload must be 14 bytes");

// IPC_IMU_SAMPLE (RPi internal)
struct ImuSamplePayload {
	uint32_t ts_ms;
	int16_t ax_mg;
	int16_t ay_mg;
	int16_t az_mg;
	int16_t gx_mdps;
	int16_t gy_mdps;
	int16_t gz_mdps;
};
static_assert(sizeof(ImuSamplePayload) == 16,
			  "ImuSamplePayload must be 16 bytes");

// IPC_DRIVE_CMD (RPi internal)
struct DriveCmdPayload {
	uint32_t ts_ms;
	int16_t steer_cdeg;
	int16_t speed_mm_s;
	uint16_t ttl_ms;
	uint8_t source;
	uint8_t flags;
};
static_assert(sizeof(DriveCmdPayload) == 12,
			  "DriveCmdPayload must be 12 bytes");

// IPC_VEHICLE_STATUS (RPi internal)
struct VehicleStatusPayload {
	uint32_t ts_ms;
	StatusPayload status;
};
static_assert(sizeof(VehicleStatusPayload) == 14,
			  "VehicleStatusPayload must be 14 bytes");

// IPC_METRICS (RPi internal)
struct MetricsPayload {
	uint32_t ts_ms;
	uint16_t cpu_temp_cdeg;
	uint16_t cpu_usage_permille;
	uint32_t mem_used_kb;
	uint32_t mem_total_kb;
};
static_assert(sizeof(MetricsPayload) == 16, "MetricsPayload must be 16 bytes");

// IPC_LOG_RECORD (RPi internal)
// text follows with length text_len (UTF-8).
struct LogRecordPayload {
	uint32_t ts_ms;
	uint8_t level;
	uint8_t text_len;
	uint8_t flags;
	uint8_t reserved;
};
static_assert(sizeof(LogRecordPayload) == 8,
			  "LogRecordPayload must be 8 bytes");

// Other commands (KILL, PING, LOG, ACK) have no fixed payload struct.

} // namespace mc::proto

#pragma pack(pop)
