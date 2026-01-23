#pragma once

#include "mc_proto.h"
#include "registry.h"

namespace mc {

static inline void send_ack(Context &ctx, uint16_t seq) {
	if (!ctx.tx)
		return;

	uint8_t out[mc::proto::MAX_FRAME_ENCODED];
	size_t out_len = 0;
	const bool ok = mc::proto::PacketWriter::build(
		out, sizeof(out), out_len, mc::proto::Type::ACK, 0, seq, nullptr, 0);
	if (ok) {
		ctx.tx->enqueue(out, (uint16_t)out_len);
	}
}

} // namespace mc
