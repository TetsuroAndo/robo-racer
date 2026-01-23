#pragma once

#include "Proto.hpp"

namespace mc::proto {

static inline bool build_ack(uint16_t seq, uint8_t *out, size_t out_cap,
							 size_t &out_len) {
	return PacketWriter::build(out, out_cap, out_len, Type::ACK, 0, seq,
							   nullptr, 0);
}

template < typename Tx > static inline bool send_ack(Tx *tx, uint16_t seq) {
	if (!tx) {
		return false;
	}
	uint8_t out[mc::proto::MAX_FRAME_ENCODED];
	size_t out_len = 0;
	if (!build_ack(seq, out, sizeof(out), out_len)) {
		return false;
	}
	return tx->enqueue(out, (uint16_t)out_len);
}

} // namespace mc::proto
