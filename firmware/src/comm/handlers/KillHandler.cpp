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
		return mc::Result::Ok();
	}
};

} // namespace

MC_REGISTER_HANDLER(mc::proto::Type::KILL, KillHandler)
