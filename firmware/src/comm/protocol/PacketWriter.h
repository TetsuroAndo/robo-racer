#pragma once

#include "Protocol.h"
#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

namespace proto {

class PacketWriter {
public:
	bool write(HardwareSerial &serial, uint8_t type, uint8_t flags, uint8_t seq,
			   const uint8_t *payload, size_t payload_len);

	bool writeRaw(HardwareSerial &serial, const Header &hdr,
				  const uint8_t *payload, size_t payload_len);
};

} // namespace proto
