#include "Handlers.h"

#include "../../config/Config.h"
#include "../Context.h"
#include "../protocol/Protocol.h"
#include <Arduino.h>
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

bool handleAutoSetpoint(const proto::FrameView &f, Context &ctx) {
	const auto *h = hdr(f);
	if (h->len != sizeof(proto::AutoSetpointPayload)) {
		if (wantsAck(h)) {
			ctx.tx.sendAck(h->type, h->seq, cfg::ACK_CODE_INVALID_PAYLOAD);
		}
		return true;
	}

	if (!ctx.safety.auto_active) {
		ctx.safety.markAutoInactive();
		if (wantsAck(h)) {
			ctx.tx.sendAck(h->type, h->seq, cfg::ACK_CODE_INVALID_TTL);
		}
		return true;
	}

	if (!ctx.autoCmd.isNewerSeq(h->seq)) {
		if (wantsAck(h)) {
			ctx.tx.sendAck(h->type, h->seq, cfg::ACK_CODE_OK);
		}
		return true;
	}

	proto::AutoSetpointPayload p{};
	memcpy(&p, payload(f), sizeof(p));
	ctx.autoCmd.set(h->seq, p, millis());

	if (wantsAck(h)) {
		ctx.tx.sendAck(h->type, h->seq, cfg::ACK_CODE_OK);
	}
	return true;
}
