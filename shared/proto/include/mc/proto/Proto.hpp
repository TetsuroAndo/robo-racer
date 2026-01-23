#pragma once

#ifdef ARDUINO
#include <Arduino.h>
#else
#include <array>
#include <cstddef>
#include <cstdint>
#endif

#include "mc_proto.h"

namespace mc::proto {

static constexpr size_t MAX_PAYLOAD = 64;
static constexpr size_t MAX_FRAME_DECODED = sizeof(Header) + MAX_PAYLOAD + 2;
static constexpr size_t MAX_FRAME_ENCODED =
	MAX_FRAME_DECODED + (MAX_FRAME_DECODED / 254) + 4;

static inline uint16_t to_le16(uint16_t v) {
#if defined(__BYTE_ORDER__) && defined(__ORDER_LITTLE_ENDIAN__) &&             \
	(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__)
	return v;
#elif defined(__BYTE_ORDER__) && defined(__ORDER_BIG_ENDIAN__) &&              \
	(__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
	return (uint16_t)((v >> 8) | (v << 8));
#else
	return v;
#endif
}
static inline uint16_t from_le16(uint16_t v) { return to_le16(v); }
static inline uint16_t host_to_le16(uint16_t v) { return to_le16(v); }
static inline uint16_t le16_to_host(uint16_t v) { return from_le16(v); }

class Frame {
public:
	const uint8_t *payload = nullptr;
	uint16_t payload_len = 0;

	uint8_t ver() const { return _hdr.ver; }
	uint8_t type() const { return _hdr.type; }
	uint8_t flags() const { return _hdr.flags; }
	uint16_t seq() const { return from_le16(_hdr.seq_le); }
	uint16_t len() const { return from_le16(_hdr.len_le); }

private:
	Header _hdr{};
	friend class PacketReader;
	friend bool decode_one(const uint8_t *enc, size_t enc_len, Frame &out,
						   uint8_t *decoded_buf, size_t decoded_cap);
};

using FrameView = Frame;

uint16_t crc16_ccitt(const uint8_t *data, size_t len);
size_t cobs_encode(const uint8_t *in, size_t len, uint8_t *out, size_t out_cap);
size_t cobs_decode(const uint8_t *in, size_t len, uint8_t *out, size_t out_cap);

class PacketWriter {
public:
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

	uint8_t _raw[MAX_FRAME_ENCODED];
	size_t _rawLen;
	uint8_t _decoded[MAX_FRAME_DECODED];
	size_t _decodedLen;
	Frame _frame;
	bool _hasFrame;
	uint32_t _badCrc;
	uint32_t _badCobs;
	uint32_t _badHdr;
};

bool decode_one(const uint8_t *enc, size_t enc_len, Frame &out,
				uint8_t *decoded_buf, size_t decoded_cap);

#ifndef ARDUINO
template < size_t N >
inline bool decode_one(const uint8_t *enc, size_t enc_len, Frame &out,
					   std::array< uint8_t, N > &decoded_buf) {
	return decode_one(enc, enc_len, out, decoded_buf.data(),
					  decoded_buf.size());
}
#endif

} // namespace mc::proto
