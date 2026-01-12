#include "comm/protocol/PacketWriter.h"
#include <string.h>

namespace mc::proto {

bool PacketEncoder::build(MsgType type, const uint8_t *payload,
						  uint16_t payload_len, uint8_t flags, uint8_t *out,
						  size_t out_cap, size_t *out_len) {
	if (!out || !out_len)
		return false;
	if (payload_len > 0 && !payload)
		return false;
	const size_t total_len = kHeaderSize + payload_len;
	if (total_len > out_cap)
		return false;

	const uint16_t len_le = payload_len;
	out[0] = VERSION;
	out[1] = static_cast< uint8_t >(type);
	out[2] = _seq++;
	out[3] = flags;
	out[4] = (uint8_t)(len_le & 0xFF);
	out[5] = (uint8_t)(len_le >> 8);

	if (payload_len > 0)
		memcpy(out + kHeaderSize, payload, payload_len);

	uint16_t crc = 0xFFFF;
	crc = crc16_ccitt_update(crc, out, 6);
	if (payload_len > 0)
		crc = crc16_ccitt_update(crc, out + kHeaderSize, payload_len);

	out[6] = (uint8_t)(crc & 0xFF);
	out[7] = (uint8_t)(crc >> 8);

	*out_len = total_len;
	return true;
}

PacketWriter::PacketWriter(Stream &out) : _out(out) {}

bool PacketWriter::send(MsgType type, const uint8_t *payload,
						uint16_t payload_len, uint8_t flags) {
	static constexpr size_t kMaxPacket = 256;
	uint8_t packet[kMaxPacket];
	size_t packet_len = 0;
	if (!_encoder.build(type, payload, payload_len, flags, packet,
						sizeof(packet), &packet_len)) {
		return false;
	}

	static constexpr size_t kMaxEncoded = kMaxPacket + (kMaxPacket / 254) + 2;
	uint8_t encoded[kMaxEncoded];
	const size_t encoded_len =
		cobsEncode(packet, packet_len, encoded, sizeof(encoded));
	if (encoded_len == 0)
		return false;

	_out.write(encoded, encoded_len);
	_out.write((uint8_t)0x00);
	return true;
}

} // namespace mc::proto
