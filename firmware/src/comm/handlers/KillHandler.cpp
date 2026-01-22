#include "Handlers.h"

#include "../Context.h"
#include "../protocol/Protocol.h"

namespace {
inline const proto::Header *hdr(const proto::FrameView &f) {
	return reinterpret_cast< const proto::Header * >(f.data);
}
inline bool wantsAck(const proto::Header *h) {
	return (h->flags & proto::FLAG_ACK_REQ) != 0;
}
} // namespace

bool handleKill(const proto::FrameView &f, Context &ctx) {
	const auto *h = hdr(f);
	if (h->len != 0) {
		if (wantsAck(h)) {
			ctx.tx.sendAck(h->type, h->seq, 3);
		}
		return true;
	}

	ctx.safety.kill_latched = true;
	if (wantsAck(h)) {
		ctx.tx.sendAck(h->type, h->seq, 0);
	}
	return true;
}

bool handleClearKill(const proto::FrameView &f, Context &ctx) {
	const auto *h = hdr(f);
	if (h->len != 0) {
		if (wantsAck(h)) {
			ctx.tx.sendAck(h->type, h->seq, 3);
		}
		return true;
	}

	ctx.safety.kill_latched = false;
	if (wantsAck(h)) {
		ctx.tx.sendAck(h->type, h->seq, 0);
	}
	return true;
}
