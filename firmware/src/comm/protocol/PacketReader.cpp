#include "PacketReader.h"

#include "Codec.h"

namespace proto {

PacketReader::PacketReader() : cobs_len_(0), last_error_(Error::kNone) {}

void PacketReader::reset() {
	cobs_len_ = 0;
	last_error_ = Error::kNone;
}

PacketReader::Result PacketReader::push(uint8_t byte, FrameView &frame) {
	if (byte != 0x00) {
		if (cobs_len_ >= sizeof(cobs_buf_)) {
			last_error_ = Error::kOverflow;
			cobs_len_ = 0;
			return Result::kError;
		}
		cobs_buf_[cobs_len_++] = byte;
		return Result::kNone;
	}

	if (cobs_len_ == 0) {
		return Result::kNone;
	}

	auto res = decodeFrame(frame);
	cobs_len_ = 0;
	return res;
}

PacketReader::Result PacketReader::decodeFrame(FrameView &frame) {
	const size_t decoded_len =
		cobs_decode(cobs_buf_, cobs_len_, frame_buf_, sizeof(frame_buf_));
	if (decoded_len == 0) {
		last_error_ = Error::kDecodeError;
		return Result::kError;
	}
	if (decoded_len < HEADER_SIZE + CRC_SIZE) {
		last_error_ = Error::kTooShort;
		return Result::kError;
	}

	const auto *hdr = reinterpret_cast< const Header * >(frame_buf_);
	const size_t payload_len = decoded_len - HEADER_SIZE - CRC_SIZE;
	if (hdr->len != payload_len || payload_len > MAX_PAYLOAD) {
		last_error_ = Error::kBadLength;
		return Result::kError;
	}

	const uint16_t crc_rx =
		static_cast< uint16_t >(frame_buf_[decoded_len - 2]) |
		(static_cast< uint16_t >(frame_buf_[decoded_len - 1]) << 8);
	const uint16_t crc_calc = crc16_ccitt(frame_buf_, decoded_len - CRC_SIZE);

	if (crc_calc != crc_rx) {
		last_error_ = Error::kCrcMismatch;
		return Result::kError;
	}

	frame.data = frame_buf_;
	frame.len = decoded_len - CRC_SIZE;
	last_error_ = Error::kNone;
	return Result::kOk;
}

} // namespace proto
