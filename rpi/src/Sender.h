#pragma once

#include "config/Config.h"
#include "proto/PacketReader.h"
#include "proto/PacketWriter.h"
#include "proto/Protocol.h"
#include "proto/Trace.h"
#include <stdint.h>

class Sender {
public:
	Sender(const char* esp_dev, int esp_baud = cfg::DEFAULT_ESP_BAUD);
	~Sender();

	void send(int speed, int angle);
	void sendAutoMode(bool enable);
	void sendKill();
	void sendClearKill();
	void poll();
private:
	void _init(const char* esp_dev, int esp_baud);
	void handleFrame(const proto::FrameView& frame);
	void handleAck(const proto::AckPayload& payload, uint8_t seq);
	void handleStatus(const proto::StatusPayload& payload);
	void maybeSendHeartbeat(uint32_t now_ms);
	uint8_t nextSeq();

	int _espFd;
	proto::PacketWriter writer_;
	proto::PacketReader reader_;
	proto::Trace trace_;
	uint8_t seq_ = 0;
	uint32_t last_hb_ms_ = 0;
	uint32_t last_status_log_ms_ = 0;
	proto::StatusPayload last_status_{};
	bool has_status_ = false;
	bool auto_enabled_ = false;
};
