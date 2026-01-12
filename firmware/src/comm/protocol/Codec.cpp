#include "comm/protocol/Codec.h"

namespace mc::proto {

uint16_t crc16_ccitt_update(uint16_t crc, const uint8_t *data, size_t len) {
	while (len--) {
		crc ^= (uint16_t)(*data++) << 8;
		for (uint8_t i = 0; i < 8; ++i) {
			if (crc & 0x8000)
				crc = (uint16_t)((crc << 1) ^ 0x1021);
			else
				crc <<= 1;
		}
	}
	return crc;
}

uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
	return crc16_ccitt_update(0xFFFF, data, len);
}

size_t cobsEncode(const uint8_t *input, size_t length, uint8_t *output,
				  size_t max_len) {
	if (!input || !output || max_len == 0)
		return 0;

	size_t read_index = 0;
	size_t write_index = 1;
	size_t code_index = 0;
	uint8_t code = 1;

	while (read_index < length) {
		if (write_index >= max_len)
			return 0;
		if (input[read_index] == 0) {
			output[code_index] = code;
			code = 1;
			code_index = write_index++;
			++read_index;
		} else {
			output[write_index++] = input[read_index++];
			++code;
			if (code == 0xFF) {
				output[code_index] = code;
				code = 1;
				code_index = write_index++;
				if (write_index > max_len)
					return 0;
			}
		}
	}

	if (code_index >= max_len)
		return 0;
	output[code_index] = code;
	return write_index;
}

} // namespace mc::proto
