#pragma once
#include <Stream.h>
#include <stdint.h>
#include "comm/protocol/Codec.h"
#include "comm/protocol/Protocol.h"

namespace mc::proto {

class PacketEncoder {
 public:
	static constexpr size_t kHeaderSize = sizeof(Header);

	bool build(MsgType type, const uint8_t *payload, uint16_t payload_len,
			   uint8_t flags, uint8_t *out, size_t out_cap,
			   size_t *out_len);

 private:
	uint8_t _seq = 0;
};

class PacketWriter {
 public:
	explicit PacketWriter(Stream &out);
	bool send(MsgType type, const uint8_t *payload, uint16_t payload_len,
			  uint8_t flags = 0);

 private:
	Stream &_out;
	PacketEncoder _encoder;
};

} // namespace mc::proto
