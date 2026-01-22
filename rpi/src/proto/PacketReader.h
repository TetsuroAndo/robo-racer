#pragma once

#include "Protocol.h"
#include <stddef.h>
#include <stdint.h>

namespace proto {

struct FrameView {
	const uint8_t *data;
	size_t len;
};

class PacketReader {
public:
	enum class Result {
		NONE,
		OK,
		ERROR,
	};

	enum class Error {
		NONE,
		BUFFER_OVERFLOW,
		DECODE_ERROR,
		TOO_SHORT,
		BAD_LENGTH,
		CRC_MISMATCH,
	};

	PacketReader();

	Result push(uint8_t byte, FrameView &frame);
	Error lastError() const { return last_error_; }
	void reset();

private:
	uint8_t cobs_buf_[MAX_ENCODED];
	size_t cobs_len_;
	uint8_t frame_buf_[MAX_FRAME];
	Error last_error_;

	Result decodeFrame(FrameView &frame);
};

} // namespace proto
