#pragma once

#include "config/Config.h"
#include <mc/ipc/UdsSeqPacket.hpp>
#include <mc/proto/Proto.hpp>
#include <stdint.h>

class Sender {
public:
	Sender(const char* sock_path);
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
	uint16_t nextSeq();

	mc::ipc::UdsClient ipc_;
	uint16_t seq_ = 0;
	uint32_t last_status_log_ms_ = 0;
	uint32_t last_hb_ms_ = 0;
	mc::proto::StatusPayload last_status_{};
	bool has_status_ = false;
	bool auto_enabled_ = false;
};
