#include "mc_proto.h"
#include <string.h>

namespace mc::proto {

static inline uint16_t le16(const uint16_t v) { return v; }
static inline uint16_t rd16(const uint8_t *p) {
	return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static inline void wr16(uint8_t *p, uint16_t v) {
	p[0] = (uint8_t)(v & 0xFF);
	p[1] = (uint8_t)(v >> 8);
}

uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
	uint16_t crc = 0xFFFF;
	for (size_t i = 0; i < len; i++) {
		crc ^= (uint16_t)data[i] << 8;
		for (int b = 0; b < 8; b++) {
			if (crc & 0x8000)
				crc = (uint16_t)((crc << 1) ^ 0x1021);
			else
				crc <<= 1;
		}
	}
	return crc;
}

size_t cobs_encode(const uint8_t *in, size_t len, uint8_t *out,
				   size_t out_cap) {
	if (!out || out_cap == 0)
		return 0;

	size_t read_index = 0;
	size_t write_index = 1;
	size_t code_index = 0;
	uint8_t code = 1;

	while (read_index < len) {
		if (write_index >= out_cap)
			return 0;

		if (in[read_index] == 0) {
			out[code_index] = code;
			code = 1;
			code_index = write_index++;
			read_index++;
		} else {
			out[write_index++] = in[read_index++];
			code++;
			if (code == 0xFF) {
				out[code_index] = code;
				code = 1;
				code_index = write_index++;
			}
		}
	}
	if (code_index >= out_cap)
		return 0;
	out[code_index] = code;
	return write_index;
}

size_t cobs_decode(const uint8_t *in, size_t len, uint8_t *out,
				   size_t out_cap) {
	if (!in || !out)
		return 0;

	size_t read_index = 0;
	size_t write_index = 0;

	while (read_index < len) {
		uint8_t code = in[read_index];
		if (code == 0)
			return 0;
		read_index++;

		for (uint8_t i = 1; i < code; i++) {
			if (read_index >= len)
				return 0;
			if (write_index >= out_cap)
				return 0;
			out[write_index++] = in[read_index++];
		}
		if (code != 0xFF && read_index < len) {
			if (write_index >= out_cap)
				return 0;
			out[write_index++] = 0;
		}
	}
	return write_index;
}

bool PacketWriter::build(uint8_t *out, size_t out_cap, size_t &out_len,
						 Type type, uint8_t flags, uint16_t seq,
						 const uint8_t *payload, uint16_t payload_len) {
	out_len = 0;
	if (payload_len > MAX_PAYLOAD)
		return false;

	uint8_t decoded[MAX_FRAME_DECODED];
	size_t decoded_len = 0;

	Header h{};
	h.magic[0] = MAGIC0;
	h.magic[1] = MAGIC1;
	h.ver = VERSION;
	h.type = (uint8_t)type;
	h.flags = flags;
	h.seq_le = le16(seq);
	h.len_le = le16(payload_len);

	memcpy(decoded, &h, sizeof(h));
	decoded_len += sizeof(h);

	if (payload_len && payload) {
		memcpy(decoded + decoded_len, payload, payload_len);
		decoded_len += payload_len;
	}

	uint16_t crc = crc16_ccitt(decoded, decoded_len);
	wr16(decoded + decoded_len, crc);
	decoded_len += 2;

	size_t enc = cobs_encode(decoded, decoded_len, out, out_cap);
	if (enc == 0)
		return false;

	if (enc + 1 > out_cap)
		return false;
	out[enc] = 0x00;
	out_len = enc + 1;
	return true;
}

PacketReader::PacketReader()
	: _rawLen(0), _decodedLen(0), _hasFrame(false), _badCrc(0), _badCobs(0),
	  _badHdr(0) {
	memset(&_frame, 0, sizeof(_frame));
}

bool PacketReader::push(uint8_t b) {
	if (_hasFrame)
		return true;

	if (b == 0x00) {
		if (_rawLen == 0)
			return false;
		bool ok = decodeFrame_();
		_rawLen = 0;
		return ok;
	}

	if (_rawLen >= sizeof(_raw)) {
		_rawLen = 0;
		_badCobs++;
		return false;
	}
	_raw[_rawLen++] = b;
	return false;
}

bool PacketReader::decodeFrame_() {
	_decodedLen = cobs_decode(_raw, _rawLen, _decoded, sizeof(_decoded));
	if (_decodedLen < sizeof(Header) + 2) {
		_badCobs++;
		return false;
	}

	uint16_t got = rd16(_decoded + _decodedLen - 2);
	uint16_t calc = crc16_ccitt(_decoded, _decodedLen - 2);
	if (got != calc) {
		_badCrc++;
		return false;
	}

	Header h{};
	memcpy(&h, _decoded, sizeof(h));

	if (h.magic[0] != MAGIC0 || h.magic[1] != MAGIC1 || h.ver != VERSION) {
		_badHdr++;
		return false;
	}

	uint16_t plen = h.len_le;
	size_t need = sizeof(Header) + (size_t)plen + 2;
	if (plen > MAX_PAYLOAD || _decodedLen != need) {
		_badHdr++;
		return false;
	}

	_frame.hdr = h;
	_frame.payload = _decoded + sizeof(Header);
	_frame.payload_len = plen;
	_hasFrame = true;
	return true;
}

void PacketReader::consumeFrame() { _hasFrame = false; }

} // namespace mc::proto
