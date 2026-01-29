#include <mc/proto/Proto.hpp>

#include <cassert>
#include <cstring>
#include <iostream>

/**
 * @brief PacketWriter->PacketReader ACK roundtrip.
 */
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
	std::cout << "\texpect type=ACK flags=0 len=0 seq=0x" << std::hex << seq
			  << std::dec << "\n";
	std::cout << "\tactual type=0x" << std::hex << (unsigned)frame.type()
			  << " flags=0x" << (unsigned)frame.flags()
			  << " len=" << frame.payload_len << " seq=0x" << frame.seq()
			  << std::dec << "\n";
	assert(frame.type() == static_cast< uint8_t >(mc::proto::Type::ACK));
	assert(frame.flags() == 0);
	assert(frame.payload_len == 0);
	assert(frame.seq() == seq);
}

/**
 * @brief PacketWriter ACK output matches expected wire bytes.
 */
static void test_writer_ack_golden_bytes() {
	std::cout << "[TEST] writer_ack_golden_bytes\n";
	uint8_t enc[mc::proto::MAX_FRAME_ENCODED];
	size_t enc_len = 0;
	const uint16_t seq = 0x1234;
	const bool ok = mc::proto::PacketWriter::build(
		enc, sizeof(enc), enc_len, mc::proto::Type::ACK, 0, seq, nullptr, 0);
	assert(ok);

	const uint8_t expected[] = {0x05, 0x4D, 0x43, 0x01, 0x80, 0x03, 0x34,
								0x12, 0x01, 0x03, 0x67, 0x80, 0x00};
	std::cout << "\tenc_len=" << enc_len << " expected=" << sizeof(expected)
			  << "\n";
	assert(enc_len == sizeof(expected));
	assert(std::memcmp(enc, expected, sizeof(expected)) == 0);
}

/**
 * @brief PacketWriter->PacketReader STATUS roundtrip (signed fields).
 */
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
	std::cout << "\texpect len=" << sizeof(payload)
			  << " actual len=" << frame.payload_len << "\n";
	assert(frame.type() == static_cast< uint8_t >(mc::proto::Type::STATUS));
	assert(frame.payload_len == sizeof(payload));
	assert(std::memcmp(frame.payload, &payload, sizeof(payload)) == 0);
}

/**
 * @brief PacketWriter->PacketReader LOG roundtrip (level + text).
 */
static void test_writer_log_roundtrip() {
	std::cout << "[TEST] writer_log_roundtrip\n";
	const uint8_t level = 2;
	const char *text = "hello-log";
	const size_t text_len = std::strlen(text);
	uint8_t payload[1 + 16]{};
	payload[0] = level;
	std::memcpy(payload + 1, text, text_len);
	const uint16_t payload_len = (uint16_t)(1 + text_len);

	uint8_t enc[mc::proto::MAX_FRAME_ENCODED];
	size_t enc_len = 0;
	const bool ok = mc::proto::PacketWriter::build(
		enc, sizeof(enc), enc_len, mc::proto::Type::LOG, 0, 0x00AA, payload,
		payload_len);
	assert(ok);

	mc::proto::PacketReader reader;
	for (size_t i = 0; i < enc_len; ++i) {
		reader.push(enc[i]);
	}
	assert(reader.hasFrame());
	const auto &frame = reader.frame();
	assert(frame.type() == static_cast< uint8_t >(mc::proto::Type::LOG));
	assert(frame.payload_len == payload_len);
	assert(frame.payload[0] == level);
	assert(std::memcmp(frame.payload + 1, text, text_len) == 0);
}

/**
 * @brief PacketWriter->PacketReader IPC_LOG_RECORD roundtrip.
 */
static void test_writer_ipc_log_record_roundtrip() {
	std::cout << "[TEST] writer_ipc_log_record_roundtrip\n";
	mc::proto::LogRecordPayload header{};
	header.ts_ms = 0x12345678u;
	header.level = 3;
	header.text_len = 6;
	header.flags = 1;
	header.reserved = 0;

	const char *text = "logrec";
	const size_t text_len = std::strlen(text);
	const uint16_t payload_len = (uint16_t)(sizeof(header) + text_len);
	uint8_t payload[64]{};
	std::memcpy(payload, &header, sizeof(header));
	std::memcpy(payload + sizeof(header), text, text_len);

	uint8_t enc[mc::proto::MAX_FRAME_ENCODED];
	size_t enc_len = 0;
	const bool ok = mc::proto::PacketWriter::build(
		enc, sizeof(enc), enc_len, mc::proto::Type::IPC_LOG_RECORD, 0, 0x00BB,
		payload, payload_len);
	assert(ok);

	mc::proto::PacketReader reader;
	for (size_t i = 0; i < enc_len; ++i) {
		reader.push(enc[i]);
	}
	assert(reader.hasFrame());
	const auto &frame = reader.frame();
	assert(frame.type() ==
		   static_cast< uint8_t >(mc::proto::Type::IPC_LOG_RECORD));
	assert(frame.payload_len == payload_len);

	mc::proto::LogRecordPayload got{};
	std::memcpy(&got, frame.payload, sizeof(got));
	assert(got.ts_ms == header.ts_ms);
	assert(got.level == header.level);
	assert(got.text_len == header.text_len);
	assert(got.flags == header.flags);
	assert(std::memcmp(frame.payload + sizeof(got), text, text_len) == 0);
}

/**
 * @brief PacketWriter rejects oversized payloads.
 */
static void test_writer_rejects_oversize() {
	std::cout << "[TEST] writer_rejects_oversize\n";
	uint8_t payload[mc::proto::MAX_PAYLOAD + 1]{};
	uint8_t enc[mc::proto::MAX_FRAME_ENCODED];
	size_t enc_len = 0;
	const bool ok = mc::proto::PacketWriter::build(
		enc, sizeof(enc), enc_len, mc::proto::Type::PING, 0, 0x0001, payload,
		(uint16_t)(mc::proto::MAX_PAYLOAD + 1));
	assert(!ok);
}

/**
 * @brief PacketReader increments badCrc on corrupt frame.
 */
static void test_reader_bad_crc() {
	std::cout << "[TEST] reader_bad_crc\n";
	uint8_t enc[mc::proto::MAX_FRAME_ENCODED];
	size_t enc_len = 0;
	const bool ok = mc::proto::PacketWriter::build(enc, sizeof(enc), enc_len,
												   mc::proto::Type::PING, 0,
												   0x1234, nullptr, 0);
	assert(ok);
	uint8_t raw[mc::proto::MAX_FRAME_DECODED];
	const size_t raw_len =
		mc::proto::cobs_decode(enc, enc_len - 1, raw, sizeof(raw));
	assert(raw_len >= sizeof(mc::proto::Header) + 2);
	raw[0] ^= 0xFF;
	uint8_t enc_bad[mc::proto::MAX_FRAME_ENCODED];
	const size_t enc_bad_len =
		mc::proto::cobs_encode(raw, raw_len, enc_bad, sizeof(enc_bad));
	assert(enc_bad_len > 0);
	enc_bad[enc_bad_len] = 0x00;
	const size_t total_len = enc_bad_len + 1;

	mc::proto::PacketReader reader;
	for (size_t i = 0; i < total_len; ++i) {
		reader.push(enc_bad[i]);
	}
	assert(!reader.hasFrame());
	assert(reader.badCrc() == 1);
}

int main() {
	test_writer_ack_roundtrip();
	test_writer_ack_golden_bytes();
	test_writer_status_roundtrip();
	test_writer_log_roundtrip();
	test_writer_ipc_log_record_roundtrip();
	test_writer_rejects_oversize();
	test_reader_bad_crc();
	std::cout << "rpi mc_proto tests ok\n";
	return 0;
}
