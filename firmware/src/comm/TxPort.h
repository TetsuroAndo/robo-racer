#pragma once

#include "protocol/PacketWriter.h"
#include "protocol/Protocol.h"
#include <Arduino.h>

class TxPort {
public:
	TxPort() = default;

	void begin(HardwareSerial &serial) { serial_ = &serial; }
	void setTrace(class ProtoTrace *trace) { trace_ = trace; }

	bool sendAck(uint8_t type_echo, uint8_t seq_echo, uint8_t code,
				 uint8_t detail = 0);
	bool sendStatus(const proto::StatusPayload &payload, uint8_t seq);

private:
	HardwareSerial *serial_ = nullptr;
	proto::PacketWriter writer_{};
	class ProtoTrace *trace_ = nullptr;
};
