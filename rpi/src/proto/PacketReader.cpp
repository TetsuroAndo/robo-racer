#include "PacketReader.h"

#include "Codec.h"

namespace proto {

PacketReader::PacketReader() : cobs_len_(0), last_error_(Error::NONE) {}

void PacketReader::reset() {
	cobs_len_ = 0;
	last_error_ = Error::NONE;
}

PacketReader::Result PacketReader::push(uint8_t byte, FrameView &frame) {
	if (byte != 0x00) {
		if (cobs_len_ >= sizeof(cobs_buf_)) {
			last_error_ = Error::BUFFER_OVERFLOW;
			cobs_len_ = 0;
			return Result::ERROR;
		}
		cobs_buf_[cobs_len_++] = byte;
		return Result::NONE;
	}

	if (cobs_len_ == 0) {
		return Result::NONE;
	}

	auto res = decodeFrame(frame);
	cobs_len_ = 0;
	return res;
}

PacketReader::Result PacketReader::decodeFrame(FrameView &frame) {
	const size_t decoded_len =
		cobs_decode(cobs_buf_, cobs_len_, frame_buf_, sizeof(frame_buf_));
	if (decoded_len == 0) {
		last_error_ = Error::DECODE_ERROR;
		return Result::ERROR;
	}
	if (decoded_len < HEADER_SIZE + CRC_SIZE) {
		last_error_ = Error::TOO_SHORT;
		return Result::ERROR;
	}

	const auto *hdr = reinterpret_cast< const Header * >(frame_buf_);
	const size_t payload_len = decoded_len - HEADER_SIZE - CRC_SIZE;
	if (hdr->len != payload_len || payload_len > MAX_PAYLOAD) {
		last_error_ = Error::BAD_LENGTH;
		return Result::ERROR;
	}

	const uint16_t crc_rx =
		static_cast< uint16_t >(frame_buf_[decoded_len - 2]) |
		(static_cast< uint16_t >(frame_buf_[decoded_len - 1]) << 8);
	const uint16_t crc_calc = crc16_ccitt(frame_buf_, decoded_len - CRC_SIZE);

	if (crc_calc != crc_rx) {
		last_error_ = Error::CRC_MISMATCH;
		return Result::ERROR;
	}

	frame.data = frame_buf_;
	frame.len = decoded_len - CRC_SIZE;
	last_error_ = Error::NONE;
	return Result::OK;
}

} // namespace proto
