#pragma once

#include "Protocol.h"
#include <stddef.h>
#include <stdint.h>

namespace proto {

class PacketWriter {
public:
	bool write(int fd, uint8_t type, uint8_t flags, uint8_t seq,
			   const uint8_t *payload, size_t payload_len);

	bool writeRaw(int fd, const Header &hdr, const uint8_t *payload,
				  size_t payload_len);
};

} // namespace proto
