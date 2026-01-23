#include <mc/proto/Proto.hpp>

#include <cassert>
#include <cstring>
#include <iostream>

static void test_writer_ack_roundtrip() {
	std::cout << "[TEST] writer_ack_roundtrip\n";
	uint8_t enc[mc::proto::MAX_FRAME_ENCODED];
	size_t enc_len = 0;
	const uint16_t seq = 0x1234;
	const bool ok = mc::proto::PacketWriter::build(
		enc, sizeof(enc), enc_len, mc::proto::Type::ACK, 0, seq, nullptr, 0);
	assert(ok);
	assert(enc_len > 0);

	mc::proto::PacketReader reader;
	for (size_t i = 0; i < enc_len; ++i) {
		reader.push(enc[i]);
	}
	assert(reader.hasFrame());
	const auto &frame = reader.frame();
	assert(frame.type() == static_cast< uint8_t >(mc::proto::Type::ACK));
	assert(frame.flags() == 0);
	assert(frame.payload_len == 0);
	assert(frame.seq() == seq);
}

static void test_writer_status_roundtrip() {
	std::cout << "[TEST] writer_status_roundtrip\n";
	mc::proto::StatusPayload payload{};
	payload.seq_applied = 0x7B;
	payload.auto_active = 1;
	payload.faults_le = mc::proto::to_le16(0x0005);
	payload.speed_mm_s_le =
		(int16_t)mc::proto::to_le16((uint16_t)(int16_t)-123);
	payload.steer_cdeg_le = (int16_t)mc::proto::to_le16((uint16_t)(int16_t)456);
	payload.age_ms_le = mc::proto::to_le16(250);

	uint8_t enc[mc::proto::MAX_FRAME_ENCODED];
	size_t enc_len = 0;
	const bool ok = mc::proto::PacketWriter::build(
		enc, sizeof(enc), enc_len, mc::proto::Type::STATUS, 0, 0x0102,
		reinterpret_cast< const uint8_t * >(&payload), sizeof(payload));
	assert(ok);

	mc::proto::PacketReader reader;
	for (size_t i = 0; i < enc_len; ++i) {
		reader.push(enc[i]);
	}
	assert(reader.hasFrame());
	const auto &frame = reader.frame();
	assert(frame.type() == static_cast< uint8_t >(mc::proto::Type::STATUS));
	assert(frame.payload_len == sizeof(payload));
	assert(std::memcmp(frame.payload, &payload, sizeof(payload)) == 0);
}

int main() {
	test_writer_ack_roundtrip();
	test_writer_status_roundtrip();
	std::cout << "rpi mc_proto tests ok\n";
	return 0;
}
