#pragma once

#include "config/Config.h"
#include <array>
#include <mc/ipc/UdsSeqPacket.hpp>
#include <mc/proto/Proto.hpp>
#include <stdint.h>
#include <unordered_map>

class TelemetryEmitter;

class Sender {
public:
	explicit Sender(const char* sock_path, TelemetryEmitter* telemetry = nullptr);
	~Sender();

	void send(int speed, int angle);
	void sendAutoMode(bool enable);
	void sendKill();
	void poll();

private:
	void _init(const char* sock_path);
	void sendHeartbeatIfDue();
	void handleFrame(const mc::proto::Frame& frame);
	void handleStatus(const mc::proto::StatusPayload& payload);
	void handleAck(uint16_t seq);
	void trackPending(uint16_t seq, const uint8_t* data, uint16_t len);
	void checkPending();
	void checkStatusLiveness();
	uint16_t nextSeq();

	mc::ipc::UdsClient ipc_;
	uint16_t seq_ = 0;
	uint32_t last_status_log_ms_ = 0;
	uint32_t last_status_ms_ = 0;
	uint32_t last_hb_ms_ = 0;
	mc::proto::StatusPayload last_status_{};
	bool has_status_ = false;
	bool status_stale_ = false;
	bool auto_enabled_ = false;
	TelemetryEmitter* telemetry_ = nullptr;

	struct PendingTx {
		uint32_t deadline_ms;
		uint8_t retries;
		uint16_t len;
		std::array< uint8_t, mc::proto::MAX_FRAME_ENCODED > data;
	};
	std::unordered_map< uint16_t, PendingTx > pending_;
};
