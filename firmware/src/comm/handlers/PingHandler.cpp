#include "../../log/AsyncLogger.h"
#include "../mc_proto.h"
#include "../registry.h"

namespace {

class PingHandler : public mc::IHandler {
public:
	mc::Result onFrame(const mc::proto::FrameView &f, mc::Context &ctx,
					   uint32_t now_ms) override {
		ctx.st->last_hb_ms = now_ms;
		if (f.payload_len != 0) {
			if (ctx.log) {
				ctx.log->log(mc::LogLevel::WARN, "proto",
							 "RX PING invalid len");
			}
			return mc::Result::Fail(mc::Errc::Invalid, "ping len");
		}
		uint8_t out[mc::proto::MAX_FRAME_ENCODED];
		size_t out_len = 0;
		const uint16_t seq = f.seq();
		mc::proto::PacketWriter::build(out, sizeof(out), out_len,
									   mc::proto::Type::ACK, 0, seq, nullptr,
									   0);
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
