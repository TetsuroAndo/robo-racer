#include "TxPort.h"

#include "ProtoTrace.h"
#include <string.h>

bool TxPort::sendAck(uint8_t type_echo, uint8_t seq_echo, uint8_t code,
					 uint8_t detail) {
	if (!serial_) {
		return false;
	}

	proto::AckPayload payload{};
	payload.type_echo = type_echo;
	payload.seq_echo = seq_echo;
	payload.code = code;
	payload.detail = detail;

	const bool ok = writer_.write(
		*serial_, static_cast< uint8_t >(proto::Type::ACK), 0, seq_echo,
		reinterpret_cast< const uint8_t * >(&payload), sizeof(payload));
	if (trace_) {
		trace_->onTx(static_cast< uint8_t >(proto::Type::ACK), seq_echo,
					 sizeof(payload), 0, ok);
	}
	return ok;
}

bool TxPort::sendStatus(const proto::StatusPayload &payload, uint8_t seq) {
	if (!serial_) {
		return false;
	}

	const bool ok = writer_.write(
		*serial_, static_cast< uint8_t >(proto::Type::STATUS), 0, seq,
		reinterpret_cast< const uint8_t * >(&payload), sizeof(payload));
	if (trace_) {
		trace_->onTx(static_cast< uint8_t >(proto::Type::STATUS), seq,
					 sizeof(payload), 0, ok);
	}
	return ok;
}
