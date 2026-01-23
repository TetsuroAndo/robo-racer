#include "comm/UartTx.h"

#include <vector>

namespace mc::test {

static std::vector< std::vector< uint8_t > > g_frames;

void reset_tx() { g_frames.clear(); }

const std::vector< std::vector< uint8_t > > &frames() { return g_frames; }

} // namespace mc::test

namespace mc {

void UartTx::begin(HardwareSerial &uart) { _uart = &uart; }

bool UartTx::enqueue(const uint8_t *data, uint16_t len) {
	if (!data || len == 0)
		return false;
	mc::test::g_frames.emplace_back(data, data + len);
	return true;
}

} // namespace mc
