#include "Handlers.h"

#include "../Context.h"
#include "../protocol/Protocol.h"
#include <Arduino.h>

namespace {
inline const proto::Header *hdr(const proto::FrameView &f) {
	return reinterpret_cast< const proto::Header * >(f.data);
}
inline bool wantsAck(const proto::Header *h) {
	return (h->flags & proto::FLAG_ACK_REQ) != 0;
}
} // namespace

bool handleHeartbeat(const proto::FrameView &f, Context &ctx) {
	const auto *h = hdr(f);
	if (h->len != 0) {
		if (wantsAck(h)) {
			ctx.tx.sendAck(h->type, h->seq, 3);
		}
		return true;
	}

	ctx.safety.last_hb_ms = millis();
	if (wantsAck(h)) {
		ctx.tx.sendAck(h->type, h->seq, 0);
	}
	return true;
}
