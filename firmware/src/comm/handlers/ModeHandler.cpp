#include "../registry.h"

namespace {

class ModeHandler : public mc::IHandler {
public:
	mc::Result onFrame(const mc::proto::FrameView &f, mc::Context &ctx,
					   uint32_t now_ms) override {
		(void)now_ms;
		if (f.payload_len < 1)
			return mc::Result::Fail(mc::Errc::Invalid, "mode len");
		uint8_t mode = f.payload[0];
		if (mode == 0)
			ctx.st->mode = mc::Mode::MANUAL;
		else if (mode == 1)
			ctx.st->mode = mc::Mode::AUTO;
		else
			return mc::Result::Fail(mc::Errc::Range, "mode val");
		return mc::Result::Ok();
	}
};

} // namespace

MC_REGISTER_HANDLER(mc::proto::Type::MODE_SET, ModeHandler)
