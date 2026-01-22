#include "../../log/AsyncLogger.h"
#include "../mc_proto.h"
#include "../registry.h"

namespace {

class PingHandler : public mc::IHandler {
public:
	mc::Result onFrame(const mc::proto::FrameView &f, mc::Context &ctx,
					   uint32_t now_ms) override {
		(void)now_ms;
		uint8_t out[mc::proto::MAX_FRAME_ENCODED];
		size_t out_len = 0;
		const uint16_t seq = (uint16_t)f.hdr.seq_le;
		mc::proto::PacketWriter::build(out, sizeof(out), out_len,
									   mc::proto::Type::ACK,
									   mc::proto::FLAG_ACK, seq, nullptr, 0);
		if (ctx.tx) {
			ctx.tx->enqueue(out, (uint16_t)out_len);
		}
		if (ctx.log) {
			ctx.log->logf(mc::LogLevel::TRACE, "proto", "RX PING -> ACK seq=%u",
						  (unsigned)seq);
		}
		return mc::Result::Ok();
	}
};

} // namespace

MC_REGISTER_HANDLER(mc::proto::Type::PING, PingHandler)
