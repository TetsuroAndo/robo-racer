#pragma once

#ifdef ARDUINO
#include <Arduino.h>
#else
#include <stddef.h>
#include <stdint.h>
#endif

#include "../../../shared/proto/mc_proto.h"

namespace mc::proto {

// Endianness helpers for this platform
static inline uint16_t le16_to_host(uint16_t v) {
#if defined(__BYTE_ORDER__) && defined(__ORDER_BIG_ENDIAN__) && \
	(__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
	return (uint16_t)((v >> 8) | (v << 8));
#else
	return v;
#endif
}

static inline uint16_t host_to_le16(uint16_t v) {
#if defined(__BYTE_ORDER__) && defined(__ORDER_BIG_ENDIAN__) && \
	(__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
	return (uint16_t)((v >> 8) | (v << 8));
#else
	return v;
#endif
}

// Platform-specific constants
static constexpr size_t   MAX_PAYLOAD = 64;
static constexpr size_t   MAX_FRAME_DECODED = sizeof(Header) + MAX_PAYLOAD + 2;
static constexpr size_t   MAX_FRAME_ENCODED = MAX_FRAME_DECODED + (MAX_FRAME_DECODED / 254) + 2;

// View of a decoded frame (for firmware use)
class FrameView {
public:
	const uint8_t* payload = nullptr;
	uint16_t payload_len = 0;

	uint8_t ver() const { return _hdr.ver; }
	uint8_t type() const { return _hdr.type; }
	uint8_t flags() const { return _hdr.flags; }
	uint16_t seq() const { return le16_to_host(_hdr.seq_le); }
	uint16_t len() const { return le16_to_host(_hdr.len_le); }

private:
	Header _hdr{};
	friend class PacketReader;
};

// Firmware-side implementation of protocol utils
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
