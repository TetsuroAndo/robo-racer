#include "TxPort.h"

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

	return writer_.write(
		*serial_, static_cast< uint8_t >(proto::Type::ACK), 0, seq_echo,
		reinterpret_cast< const uint8_t * >(&payload), sizeof(payload));
}

bool TxPort::sendStatus(const proto::StatusPayload &payload, uint8_t seq) {
	if (!serial_) {
		return false;
	}

	return writer_.write(*serial_, static_cast< uint8_t >(proto::Type::STATUS),
						 0, seq, reinterpret_cast< const uint8_t * >(&payload),
						 sizeof(payload));
}
