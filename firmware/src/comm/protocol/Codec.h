#pragma once
#include <stddef.h>
#include <stdint.h>

namespace mc::proto {

uint16_t crc16_ccitt_update(uint16_t crc, const uint8_t *data, size_t len);
uint16_t crc16_ccitt(const uint8_t *data, size_t len);

// Returns encoded length, or 0 if output buffer is too small.
size_t cobsEncode(const uint8_t *input, size_t length, uint8_t *output,
				  size_t max_len);

} // namespace mc::proto
