#include "Handlers.h"

#include "../Context.h"
#include "../protocol/Protocol.h"
#include <string.h>

namespace {
inline const proto::Header *hdr(const proto::FrameView &f) {
	return reinterpret_cast< const proto::Header * >(f.data);
}
inline const uint8_t *payload(const proto::FrameView &f) {
	return f.data + sizeof(proto::Header);
}
inline bool wantsAck(const proto::Header *h) {
	return (h->flags & proto::FLAG_ACK_REQ) != 0;
}
} // namespace

bool handleAutoMode(const proto::FrameView &f, Context &ctx) {
	const auto *h = hdr(f);
	if (h->len != sizeof(proto::AutoModePayload)) {
		if (wantsAck(h)) {
			ctx.tx.sendAck(h->type, h->seq, 3);
		}
		return true;
	}

	proto::AutoModePayload p{};
	memcpy(&p, payload(f), sizeof(p));

	ctx.safety.auto_active = (p.enable != 0);
	if (!ctx.safety.auto_active) {
		ctx.autoCmd.has = false;
	}

	if (wantsAck(h)) {
		ctx.tx.sendAck(h->type, h->seq, 0);
	}
	return true;
}
