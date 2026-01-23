#include <mc/proto/Proto.hpp>

#include <cstring>

namespace mc::proto {

uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
	uint16_t crc = 0xFFFF;
	for (size_t i = 0; i < len; ++i) {
		crc ^= (uint16_t)data[i] << 8;
		for (int b = 0; b < 8; ++b) {
			if (crc & 0x8000)
				crc = (uint16_t)((crc << 1) ^ 0x1021);
			else
				crc = (uint16_t)(crc << 1);
		}
	}
	return crc;
}

size_t cobs_encode(const uint8_t *in, size_t len, uint8_t *out,
				   size_t out_cap) {
	if (!out || out_cap == 0)
		return 0;
	size_t read_index = 0, write_index = 1, code_index = 0;
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
			if (++code == 0xFF) {
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
	size_t read_index = 0, write_index = 0;
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

static inline void wr16(uint8_t *p, uint16_t v_le) {
	const uint16_t v = v_le;
	p[0] = (uint8_t)(v & 0xFF);
	p[1] = (uint8_t)(v >> 8);
}

static inline uint16_t rd16(const uint8_t *p) {
	return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

bool PacketWriter::build(uint8_t *out, size_t out_cap, size_t &out_len,
						 Type type, uint8_t flags, uint16_t seq,
						 const uint8_t *payload, uint16_t payload_len) {
	out_len = 0;
	if (payload_len > MAX_PAYLOAD)
		return false;

	std::array< uint8_t, MAX_FRAME_DECODED > decoded{};
	size_t decoded_len = 0;

	Header h{};
	h.magic[0] = MAGIC0;
	h.magic[1] = MAGIC1;
	h.ver = VERSION;
	h.type = (uint8_t)type;
	h.flags = flags;
	h.seq_le = to_le16(seq);
	h.len_le = to_le16(payload_len);

	std::memcpy(decoded.data(), &h, sizeof(h));
	decoded_len += sizeof(h);

	if (payload_len && payload) {
		std::memcpy(decoded.data() + decoded_len, payload, payload_len);
		decoded_len += payload_len;
	}

	const uint16_t crc = crc16_ccitt(decoded.data(), decoded_len);
	wr16(decoded.data() + decoded_len, to_le16(crc));
	decoded_len += 2;

	const size_t enc = cobs_encode(decoded.data(), decoded_len, out, out_cap);
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
	_frame = Frame{};
}

bool PacketReader::push(uint8_t b) {
	if (_hasFrame)
		return true;

	if (b == 0x00) {
		if (_rawLen == 0)
			return false;
		const bool ok = decodeFrame_();
		_rawLen = 0;
		return ok && _hasFrame;
	}

	if (_rawLen >= _raw.size()) {
		_rawLen = 0;
		_badCobs++;
		return false;
	}
	_raw[_rawLen++] = b;
	return false;
}

bool PacketReader::decodeFrame_() {
	_decodedLen =
		cobs_decode(_raw.data(), _rawLen, _decoded.data(), _decoded.size());
	if (_decodedLen < sizeof(Header) + 2) {
		_badCobs++;
		return false;
	}

	const uint16_t got_crc = from_le16(rd16(_decoded.data() + _decodedLen - 2));
	const uint16_t calc_crc = crc16_ccitt(_decoded.data(), _decodedLen - 2);
	if (got_crc != calc_crc) {
		_badCrc++;
		return false;
	}

	Header h{};
	std::memcpy(&h, _decoded.data(), sizeof(h));
	if (h.magic[0] != MAGIC0 || h.magic[1] != MAGIC1 || h.ver != VERSION) {
		_badHdr++;
		return false;
	}

	const uint16_t plen = from_le16(h.len_le);
	const size_t need = sizeof(Header) + (size_t)plen + 2;
	if (plen > MAX_PAYLOAD || _decodedLen != need) {
		_badHdr++;
		return false;
	}

	_frame._hdr = h;
	_frame.payload = _decoded.data() + sizeof(Header);
	_frame.payload_len = plen;
	_hasFrame = true;
	return true;
}

void PacketReader::consumeFrame() { _hasFrame = false; }

bool decode_one(const uint8_t *enc, size_t enc_len, Frame &out,
				std::array< uint8_t, MAX_FRAME_DECODED > &decoded_buf) {
	if (!enc || enc_len == 0)
		return false;
	if (enc[enc_len - 1] == 0x00)
		enc_len--;

	const size_t dec_len =
		cobs_decode(enc, enc_len, decoded_buf.data(), decoded_buf.size());
	if (dec_len < sizeof(Header) + 2)
		return false;

	Header h{};
	std::memcpy(&h, decoded_buf.data(), sizeof(Header));
	if (h.magic[0] != MAGIC0 || h.magic[1] != MAGIC1 || h.ver != VERSION) {
		return false;
	}

	const uint16_t payload_len = from_le16(h.len_le);
	const size_t frame_wo_crc = sizeof(Header) + payload_len;
	if (frame_wo_crc + 2 != dec_len)
		return false;

	uint16_t crc_le = 0;
	std::memcpy(&crc_le, decoded_buf.data() + frame_wo_crc, 2);
	const uint16_t crc = from_le16(crc_le);
	const uint16_t expect = crc16_ccitt(decoded_buf.data(), frame_wo_crc);
	if (crc != expect)
		return false;

	out._hdr = h;
	out.payload = decoded_buf.data() + sizeof(Header);
	out.payload_len = payload_len;
	return true;
}

} // namespace mc::proto
