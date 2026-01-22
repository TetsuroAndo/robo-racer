#include "Codec.h"

namespace proto {

uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
	uint16_t crc = 0xFFFF;
	for (size_t i = 0; i < len; ++i) {
		crc ^= static_cast< uint16_t >(data[i]) << 8;
		for (uint8_t bit = 0; bit < 8; ++bit) {
			if (crc & 0x8000) {
				crc = static_cast< uint16_t >((crc << 1) ^ 0x1021);
			} else {
				crc <<= 1;
			}
		}
	}
	return crc;
}

size_t cobs_encode(const uint8_t *input, size_t length, uint8_t *output,
				   size_t output_max) {
	if (output_max == 0) {
		return 0;
	}

	size_t read_index = 0;
	size_t write_index = 1;
	size_t code_index = 0;
	uint8_t code = 1;

	while (read_index < length) {
		if (input[read_index] == 0) {
			if (write_index >= output_max) {
				return 0;
			}
			output[code_index] = code;
			code = 1;
			code_index = write_index++;
			++read_index;
			continue;
		}

		if (write_index >= output_max) {
			return 0;
		}

		output[write_index++] = input[read_index++];
		++code;

		if (code == 0xFF) {
			if (write_index >= output_max) {
				return 0;
			}
			output[code_index] = code;
			code = 1;
			code_index = write_index++;
		}
	}

	output[code_index] = code;
	return write_index;
}

size_t cobs_decode(const uint8_t *input, size_t length, uint8_t *output,
				   size_t output_max) {
	size_t read_index = 0;
	size_t write_index = 0;

	while (read_index < length) {
		uint8_t code = input[read_index];
		if (code == 0) {
			return 0;
		}
		++read_index;

		for (uint8_t i = 1; i < code; ++i) {
			if (read_index >= length || write_index >= output_max) {
				return 0;
			}
			output[write_index++] = input[read_index++];
		}

		if (code != 0xFF && read_index < length) {
			if (write_index >= output_max) {
				return 0;
			}
			output[write_index++] = 0;
		}
	}

	return write_index;
}

} // namespace proto
