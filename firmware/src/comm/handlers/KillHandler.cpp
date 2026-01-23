#include "../../log/AsyncLogger.h"
#include "../Ack.h"
#include "../registry.h"

namespace {

class KillHandler : public mc::IHandler {
public:
	mc::Result onFrame(const mc::proto::FrameView &f, mc::Context &ctx,
					   uint32_t now_ms) override {
		(void)f;
		(void)now_ms;
		ctx.st->killed = true;
		ctx.st->cmd_expire_ms = 0;
		if (ctx.log) {
			ctx.log->log(mc::LogLevel::WARN, "proto", "RX KILL -> killed=1");
		}
		if (f.flags() & mc::proto::FLAG_ACK_REQ) {
			const uint16_t seq = f.seq();
			mc::send_ack(ctx, seq);
		}
		return mc::Result::Ok();
	}
};

} // namespace

MC_REGISTER_HANDLER(mc::proto::Type::KILL, KillHandler)
