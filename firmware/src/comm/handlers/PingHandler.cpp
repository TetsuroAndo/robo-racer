#include "../../log/AsyncLogger.h"
#include "../registry.h"
#include <mc/proto/Ack.hpp>

namespace {

class PingHandler : public mc::IHandler {
public:
	mc::Result onFrame(const mc::proto::FrameView &f, mc::Context &ctx,
					   uint32_t now_ms) override {
		ctx.st->last_hb_ms = now_ms;
		if (f.payload_len != 0) {
			if (ctx.log) {
				ctx.log->log(mc::LogLevel::WARN, "proto",
							 "RX PING nonzero len");
			}
			return mc::Result::Fail(mc::Errc::Invalid, "ping len");
		}
		const uint16_t seq = f.seq();
		mc::proto::send_ack(ctx.tx, seq);
		if (ctx.log) {
			ctx.log->logf(mc::LogLevel::TRACE, "proto", "RX PING -> ACK seq=%u",
						  (unsigned)seq);
		}
		return mc::Result::Ok();
	}
};

} // namespace

MC_REGISTER_HANDLER(mc::proto::Type::PING, PingHandler)
