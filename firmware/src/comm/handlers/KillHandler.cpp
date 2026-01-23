#include "../../log/AsyncLogger.h"
#include "../registry.h"
#include <mc/proto/Ack.hpp>

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
			mc::proto::send_ack(ctx.tx, f.seq());
		}
		return mc::Result::Ok();
	}
};

} // namespace

MC_REGISTER_HANDLER(mc::proto::Type::KILL, KillHandler)
