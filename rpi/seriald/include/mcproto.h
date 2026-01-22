#pragma once
#include <stdint.h>
#include <stddef.h>

namespace mc::proto {

static constexpr uint8_t  MAGIC0 = 'M';
static constexpr uint8_t  MAGIC1 = 'C';
static constexpr uint8_t  VERSION = 1;

static constexpr size_t   MAX_PAYLOAD = 64;
static constexpr size_t   MAX_FRAME_DECODED = 2 + 1 + 1 + 1 + 2 + 2 + MAX_PAYLOAD + 2;
static constexpr size_t   MAX_FRAME_ENCODED = MAX_FRAME_DECODED + (MAX_FRAME_DECODED / 254) + 2;

enum class Type : uint8_t {
	DRIVE     = 0x01,
	KILL      = 0x02,
	MODE_SET  = 0x03,
	PING      = 0x04,
	LOG       = 0x10,
	STATUS    = 0x11,
	ACK       = 0x80,
};

enum : uint8_t {
	FLAG_ACK_REQ = 1u << 0,
	FLAG_ACK     = 1u << 1,
	FLAG_ERR     = 1u << 2,
};

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

struct FrameView {
	Header hdr;
	const uint8_t* payload;
	uint16_t payload_len;
};

uint16_t crc16_ccitt(const uint8_t* data, size_t len);

size_t cobs_encode(const uint8_t* in, size_t len, uint8_t* out, size_t out_cap);
size_t cobs_decode(const uint8_t* in, size_t len, uint8_t* out, size_t out_cap);

class PacketWriter {
public:
	static bool build(uint8_t* out, size_t out_cap, size_t& out_len,
					  Type type, uint8_t flags, uint16_t seq,
					  const uint8_t* payload, uint16_t payload_len);
};

class PacketReader {
public:
	PacketReader();
	bool push(uint8_t b);
	bool hasFrame() const { return _hasFrame; }
	const FrameView& frame() const { return _frame; }
	void consumeFrame();

	uint32_t badCrc() const { return _badCrc; }
	uint32_t badCobs() const { return _badCobs; }
	uint32_t badHdr() const { return _badHdr; }

private:
	uint8_t _raw[MAX_FRAME_ENCODED];
	size_t  _rawLen;

	uint8_t _decoded[MAX_FRAME_DECODED];
	size_t  _decodedLen;

	bool _hasFrame;
	FrameView _frame;

	uint32_t _badCrc, _badCobs, _badHdr;

	bool decodeFrame_();
};

} // namespace mc::proto
