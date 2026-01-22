#pragma once

#include "protocol/PacketReader.h"
#include <stdint.h>

struct Context;

using HandlerFn = bool (*)(const proto::FrameView &, Context &);

struct Dispatcher {
	HandlerFn table[256]{};

	void reg(uint8_t type, HandlerFn fn) { table[type] = fn; }

	bool dispatch(uint8_t type, const proto::FrameView &frame, Context &ctx) {
		auto fn = table[type];
		if (!fn) {
			return false;
		}
		return fn(frame, ctx);
	}
};
