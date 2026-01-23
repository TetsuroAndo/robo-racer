#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace mc::proto {

static constexpr uint8_t MAGIC0 = 'M';
static constexpr uint8_t MAGIC1 = 'C';
static constexpr uint8_t VERSION = 1;

static constexpr size_t MAX_PAYLOAD = 64;

#pragma pack(push, 1)
struct Header {
	uint8_t magic[2];
	uint8_t ver;
	uint8_t type;
	uint8_t flags;
	uint16_t seq_le;
	uint16_t len_le;
};
#pragma pack(pop)
static_assert(sizeof(Header) == 9, "Header must be 9 bytes on wire");

enum class Type : uint8_t {
	DRIVE = 0x01,
	KILL = 0x02,
	MODE_SET = 0x03,
	PING = 0x04,

	LOG = 0x10,
	STATUS = 0x11,
	HILS_STATE = 0x12,
	ACK = 0x80,
};

enum : uint8_t {
	FLAG_ACK_REQ = 0x01,
	FLAG_ACK = 0x02,
	FLAG_ERR = 0x04,
};

static constexpr size_t MAX_FRAME_DECODED = sizeof(Header) + MAX_PAYLOAD + 2;
static constexpr size_t MAX_FRAME_ENCODED =
	MAX_FRAME_DECODED + (MAX_FRAME_DECODED / 254) + 4;

struct Frame {
	Header hdr;
	const uint8_t *payload;
	uint16_t payload_len;
};

uint16_t crc16_ccitt(const uint8_t *data, size_t len);

size_t cobs_encode(const uint8_t *in, size_t len, uint8_t *out, size_t out_cap);
size_t cobs_decode(const uint8_t *in, size_t len, uint8_t *out, size_t out_cap);

struct PacketWriter {
	static bool build(uint8_t *out, size_t out_cap, size_t &out_len, Type type,
					  uint8_t flags, uint16_t seq, const uint8_t *payload,
					  uint16_t payload_len);
};

class PacketReader {
public:
	PacketReader();
	bool push(uint8_t b);
	bool hasFrame() const { return _hasFrame; }
	const Frame &frame() const { return _frame; }
	void consumeFrame();

	uint32_t badCrc() const { return _badCrc; }
	uint32_t badCobs() const { return _badCobs; }
	uint32_t badHdr() const { return _badHdr; }

private:
	bool decodeFrame_();

	std::array< uint8_t, MAX_FRAME_ENCODED > _raw;
	size_t _rawLen;
	std::array< uint8_t, MAX_FRAME_DECODED > _decoded;
	size_t _decodedLen;
	Frame _frame;
	bool _hasFrame;
	uint32_t _badCrc;
	uint32_t _badCobs;
	uint32_t _badHdr;
};

bool decode_one(const uint8_t *enc, size_t enc_len, Frame &out,
				std::array< uint8_t, MAX_FRAME_DECODED > &decoded_buf);

static inline uint16_t to_le16(uint16_t v) {
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
	return v;
#else
	return (uint16_t)((v >> 8) | (v << 8));
#endif
}
static inline uint16_t from_le16(uint16_t v) { return to_le16(v); }

} // namespace mc::proto
