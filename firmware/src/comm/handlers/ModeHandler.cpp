#include "../../log/AsyncLogger.h"
#include "../registry.h"
#include <mc/proto/Ack.hpp>

namespace {

class ModeHandler : public mc::IHandler {
public:
	mc::Result onFrame(const mc::proto::FrameView &f, mc::Context &ctx,
					   uint32_t now_ms) override {
		(void)now_ms;
		if (f.payload_len != 1) {
			if (ctx.log) {
				ctx.log->log(mc::LogLevel::WARN, "proto",
							 "RX MODE invalid len");
			}
			return mc::Result::Fail(mc::Errc::Invalid, "mode len");
		}
		uint8_t mode = f.payload[0];
		if (mode == 0)
			ctx.st->mode = mc::Mode::MANUAL;
		else if (mode == 1)
			ctx.st->mode = mc::Mode::AUTO;
		else {
			if (ctx.log) {
				ctx.log->logf(mc::LogLevel::WARN, "proto",
							  "RX MODE invalid val=%u", (unsigned)mode);
			}
			return mc::Result::Fail(mc::Errc::Range, "mode val");
		}
		if (ctx.log) {
			ctx.log->logf(mc::LogLevel::INFO, "proto", "RX MODE -> %s",
						  (mode == 0 ? "MANUAL" : "AUTO"));
		}
		if (f.flags() & mc::proto::FLAG_ACK_REQ) {
			mc::proto::send_ack(ctx.tx, f.seq());
		}
		return mc::Result::Ok();
	}
};

} // namespace

MC_REGISTER_HANDLER(mc::proto::Type::MODE_SET, ModeHandler)
