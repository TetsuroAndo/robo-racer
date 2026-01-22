#include "PacketWriter.h"

#include "Codec.h"
#include <string.h>
#include <unistd.h>

namespace proto {
namespace {
bool writeAll(int fd, const uint8_t *data, size_t len) {
	size_t sent = 0;
	while (sent < len) {
		ssize_t res = write(fd, data + sent, len - sent);
		if (res <= 0) {
			return false;
		}
		sent += static_cast< size_t >(res);
	}
	return true;
}
} // namespace

bool PacketWriter::write(int fd, uint8_t type, uint8_t flags, uint8_t seq,
						 const uint8_t *payload, size_t payload_len) {
	Header hdr{};
	hdr.ver = VER;
	hdr.type = type;
	hdr.flags = flags;
	hdr.seq = seq;
	hdr.len = static_cast< uint16_t >(payload_len);
	return writeRaw(fd, hdr, payload, payload_len);
}

bool PacketWriter::writeRaw(int fd, const Header &hdr, const uint8_t *payload,
							size_t payload_len) {
	if (payload_len > MAX_PAYLOAD) {
		return false;
	}

	uint8_t frame_buf[MAX_FRAME];
	uint8_t cobs_buf[MAX_ENCODED];

	memcpy(frame_buf, &hdr, sizeof(Header));
	if (payload_len > 0 && payload != nullptr) {
		memcpy(frame_buf + sizeof(Header), payload, payload_len);
	}

	const size_t frame_len = sizeof(Header) + payload_len;
	const uint16_t crc = crc16_ccitt(frame_buf, frame_len);
	frame_buf[frame_len] = static_cast< uint8_t >(crc & 0xFF);
	frame_buf[frame_len + 1] = static_cast< uint8_t >((crc >> 8) & 0xFF);

	const size_t encoded_len = cobs_encode(frame_buf, frame_len + CRC_SIZE,
										   cobs_buf, sizeof(cobs_buf));
	if (encoded_len == 0) {
		return false;
	}

	if (!writeAll(fd, cobs_buf, encoded_len)) {
		return false;
	}
	const uint8_t terminator = 0x00;
	return writeAll(fd, &terminator, 1);
}

} // namespace proto
