#pragma once

#include <stddef.h>
#include <stdint.h>

namespace proto {

uint16_t crc16_ccitt(const uint8_t *data, size_t len);

size_t cobs_encode(const uint8_t *input, size_t length, uint8_t *output,
				   size_t output_max);
size_t cobs_decode(const uint8_t *input, size_t length, uint8_t *output,
				   size_t output_max);

} // namespace proto
