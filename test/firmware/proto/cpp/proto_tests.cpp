#include "mc_proto.h"

#include <cassert>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <vector>

namespace {
static constexpr size_t kCrcSize = 2;

size_t build_raw_frame(const mc::proto::Header &hdr, const uint8_t *payload,
					   size_t payload_len, uint8_t *out, size_t out_max) {
	const size_t frame_len = sizeof(mc::proto::Header) + payload_len;
	const size_t total_len = frame_len + kCrcSize;
	if (total_len > out_max) {
		return 0;
	}
	std::memcpy(out, &hdr, sizeof(mc::proto::Header));
	if (payload_len > 0 && payload != nullptr) {
		std::memcpy(out + sizeof(mc::proto::Header), payload, payload_len);
	}
	const uint16_t crc = mc::proto::crc16_ccitt(out, frame_len);
	out[frame_len] = static_cast< uint8_t >(crc & 0xFF);
	out[frame_len + 1] = static_cast< uint8_t >((crc >> 8) & 0xFF);
	return total_len;
}

std::vector< uint8_t > encode_frame(const uint8_t *raw, size_t raw_len) {
	uint8_t cobs_buf[mc::proto::MAX_FRAME_ENCODED];
	const size_t encoded_len =
		mc::proto::cobs_encode(raw, raw_len, cobs_buf, sizeof(cobs_buf));
	assert(encoded_len > 0);
	std::vector< uint8_t > out(cobs_buf, cobs_buf + encoded_len);
	out.push_back(0x00);
	return out;
}

void print_bytes(const char *label, const uint8_t *data, size_t len) {
	std::cout << "\t" << label << " (" << len << "): ";
	for (size_t i = 0; i < len; ++i) {
		std::cout << std::hex << std::setw(2) << std::setfill('0')
				  << static_cast< unsigned int >(data[i]);
		if (i + 1 != len) {
			std::cout << " ";
		}
	}
	std::cout << std::dec << "\n";
}
} // namespace

/**
 * @brief CRC16-CCITT の既知ベクトルで実装の正当性を検証する。
 *
 * "123456789" のCRCが 0x29B1 になることを確認し、
 * 多項式・初期値・ビット処理が期待通りであるかを担保する。
 */
static void test_crc16() {
	std::cout << "[TEST] crc16\n";
	const uint8_t data[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
	const uint16_t crc = mc::proto::crc16_ccitt(data, sizeof(data));
	std::cout << "\tgot: 0x" << std::hex << std::setw(4) << std::setfill('0')
			  << crc << std::dec << "\n";
	std::cout << "\texpected: 0x29b1\n";
	assert(crc == 0x29B1);
}

/**
 * @brief COBSエンコード/デコードの往復でデータが失われないことを確認する。
 *
 * 0x00 を含む入力が、エンコード後にデコードして完全一致することを検証し、
 * フレーミング境界（0x00区切り）が壊れないことを確かめる。
 */
static void test_cobs_roundtrip() {
	std::cout << "[TEST] cobs_roundtrip\n";
	const uint8_t input[] = {0x11, 0x00, 0x22, 0x33, 0x00, 0x44};
	uint8_t encoded[32];
	uint8_t decoded[32];

	const size_t enc_len =
		mc::proto::cobs_encode(input, sizeof(input), encoded, sizeof(encoded));
	assert(enc_len > 0);
	const size_t dec_len =
		mc::proto::cobs_decode(encoded, enc_len, decoded, sizeof(decoded));
	print_bytes("input", input, sizeof(input));
	print_bytes("encoded", encoded, enc_len);
	print_bytes("decoded", decoded, dec_len);
	assert(dec_len == sizeof(input));
	assert(std::memcmp(input, decoded, sizeof(input)) == 0);
}

/**
 * @brief 正常なフレームを受信したときにPacketReaderがOKになることを検証する。
 *
 * Header/CRCを含むフレームをCOBS化して投入し、復元されたHeaderが
 * 送信元と一致することを確認する。
 */
static void test_reader_ok() {
	std::cout << "[TEST] packet_reader_ok\n";
	mc::proto::Header hdr{};
	hdr.magic[0] = mc::proto::MAGIC0;
	hdr.magic[1] = mc::proto::MAGIC1;
	hdr.ver = mc::proto::VERSION;
	hdr.type = static_cast< uint8_t >(mc::proto::Type::PING);
	hdr.flags = 0;
	hdr.seq_le = 0x0042;
	hdr.len_le = 2;
	const uint8_t payload[2] = {0x01, 0x00};

	uint8_t raw[mc::proto::MAX_FRAME_DECODED];
	const size_t raw_len =
		build_raw_frame(hdr, payload, sizeof(payload), raw, sizeof(raw));
	assert(raw_len > 0);

	const auto encoded = encode_frame(raw, raw_len);
	mc::proto::PacketReader reader;
	for (uint8_t b : encoded) {
		reader.push(b);
	}
	assert(reader.hasFrame());
	const auto &frame = reader.frame();
	std::cout << "\theader got: ver=" << static_cast< int >(frame.ver())
			  << " type=0x" << std::hex << static_cast< int >(frame.type())
			  << " flags=0x" << static_cast< int >(frame.flags()) << " seq=0x"
			  << static_cast< int >(frame.seq()) << std::dec
			  << " len=" << frame.len() << "\n";
	std::cout << "\theader expected: ver=" << static_cast< int >(hdr.ver)
			  << " type=0x" << std::hex << static_cast< int >(hdr.type)
			  << " flags=0x" << static_cast< int >(hdr.flags) << " seq=0x"
			  << static_cast< int >(mc::proto::le16_to_host(hdr.seq_le))
			  << std::dec << " len=" << mc::proto::le16_to_host(hdr.len_le)
			  << "\n";
	assert(frame.ver() == mc::proto::VERSION);
	assert(frame.type() == hdr.type);
	assert(frame.flags() == hdr.flags);
	assert(frame.seq() == mc::proto::le16_to_host(hdr.seq_le));
	assert(frame.len() == mc::proto::le16_to_host(hdr.len_le));
	assert(frame.payload_len == sizeof(payload));
	assert(std::memcmp(frame.payload, payload, sizeof(payload)) == 0);
}

/**
 * @brief CRCが壊れているフレームをPacketReaderが検出することを検証する。
 *
 * 1バイトを反転させたフレームを送信し、badCrcが増えることを確認する。
 */
static void test_reader_crc_fail() {
	std::cout << "[TEST] packet_reader_crc_fail\n";
	mc::proto::Header hdr{};
	hdr.magic[0] = mc::proto::MAGIC0;
	hdr.magic[1] = mc::proto::MAGIC1;
	hdr.ver = mc::proto::VERSION;
	hdr.type = static_cast< uint8_t >(mc::proto::Type::DRIVE);
	hdr.flags = 0;
	hdr.seq_le = 0x0010;
	hdr.len_le = 1;
	uint8_t payload[1] = {0x7A};

	uint8_t raw[mc::proto::MAX_FRAME_DECODED];
	const size_t raw_len =
		build_raw_frame(hdr, payload, sizeof(payload), raw, sizeof(raw));
	assert(raw_len > 0);

	raw[sizeof(mc::proto::Header)] ^= 0xFF;
	const auto encoded = encode_frame(raw, raw_len);

	mc::proto::PacketReader reader;
	for (uint8_t b : encoded) {
		reader.push(b);
	}
	std::cout << "\tbadCrc: " << reader.badCrc() << "\n";
	assert(reader.badCrc() == 1);
	assert(!reader.hasFrame());
}

/**
 * @brief ヘッダのlenと実際のpayload長が不一致のフレームを検出する。
 *
 * lenを意図的に不正値にし、badHdrが増えることを確認する。
 */
static void test_reader_bad_length() {
	std::cout << "[TEST] packet_reader_bad_length\n";
	mc::proto::Header hdr{};
	hdr.magic[0] = mc::proto::MAGIC0;
	hdr.magic[1] = mc::proto::MAGIC1;
	hdr.ver = mc::proto::VERSION;
	hdr.type = static_cast< uint8_t >(mc::proto::Type::MODE_SET);
	hdr.flags = 0;
	hdr.seq_le = 0x0020;
	hdr.len_le = 3;
	const uint8_t payload[2] = {0x00, 0x00};

	uint8_t raw[mc::proto::MAX_FRAME_DECODED];
	const size_t raw_len =
		build_raw_frame(hdr, payload, sizeof(payload), raw, sizeof(raw));
	assert(raw_len > 0);
	const auto encoded = encode_frame(raw, raw_len);

	mc::proto::PacketReader reader;
	for (uint8_t b : encoded) {
		reader.push(b);
	}
	std::cout << "\tbadHdr: " << reader.badHdr() << "\n";
	assert(reader.badHdr() == 1);
	assert(!reader.hasFrame());
}

/**
 * @brief プロトコルバージョンが異なるフレームを受信できないことを確認する。
 *
 * Header上のverが期待値ではないため、badHdrが増えることを確認する。
 */
static void test_reader_bad_version() {
	std::cout << "[TEST] packet_reader_bad_version\n";
	mc::proto::Header hdr{};
	hdr.magic[0] = mc::proto::MAGIC0;
	hdr.magic[1] = mc::proto::MAGIC1;
	hdr.ver = static_cast< uint8_t >(mc::proto::VERSION + 1);
	hdr.type = static_cast< uint8_t >(mc::proto::Type::PING);
	hdr.flags = 0x01;
	hdr.seq_le = 0x0033;
	hdr.len_le = 0;

	uint8_t raw[mc::proto::MAX_FRAME_DECODED];
	const size_t raw_len = build_raw_frame(hdr, nullptr, 0, raw, sizeof(raw));
	assert(raw_len > 0);
	const auto encoded = encode_frame(raw, raw_len);

	mc::proto::PacketReader reader;
	for (uint8_t b : encoded) {
		reader.push(b);
	}
	std::cout << "\tbadHdr: " << reader.badHdr() << "\n";
	assert(reader.badHdr() == 1);
	assert(!reader.hasFrame());
}

/**
 * @brief CRC/フレーム生成の最終バイト列が期待値と一致することを確認する。
 *
 * 固定のHeader/ペイロードから生成されるrawフレームが
 * 既知のゴールデンバイト列と一致することを検証する。
 */
static void test_golden_frame_output() {
	std::cout << "[TEST] golden_frame_output\n";
	mc::proto::Header hdr{};
	hdr.magic[0] = mc::proto::MAGIC0;
	hdr.magic[1] = mc::proto::MAGIC1;
	hdr.ver = 0x01;
	hdr.type = static_cast< uint8_t >(mc::proto::Type::ACK);
	hdr.flags = 0x01;
	hdr.seq_le = 0x0010;
	hdr.len_le = 0x0004;
	const uint8_t payload[4] = {0xAA, 0xBB, 0xCC, 0xDD};

	uint8_t raw[mc::proto::MAX_FRAME_DECODED];
	const size_t raw_len =
		build_raw_frame(hdr, payload, sizeof(payload), raw, sizeof(raw));

	const uint8_t expected[] = {0x4D, 0x43, 0x01, 0x80, 0x01, 0x10, 0x00, 0x04,
								0x00, 0xAA, 0xBB, 0xCC, 0xDD, 0xE6, 0xC8};
	print_bytes("raw", raw, raw_len);
	print_bytes("expected", expected, sizeof(expected));
	assert(raw_len == sizeof(expected));
	assert(std::memcmp(raw, expected, sizeof(expected)) == 0);
}

/**
 * @brief PacketWriterでACKを生成し、PacketReaderで復元できることを確認する。
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
	assert(frame.type() == static_cast< uint8_t >(mc::proto::Type::ACK));
	assert(frame.flags() == 0);
	assert(frame.payload_len == 0);
	assert(frame.seq() == seq);
}

/**
 * @brief PacketWriterでSTATUSを生成し、payloadが保持されることを確認する。
 */
static void test_writer_status_roundtrip() {
	std::cout << "[TEST] writer_status_roundtrip\n";
	mc::proto::StatusPayload payload{};
	payload.seq_applied = 0x7B;
	payload.auto_active = 1;
	payload.faults_le = mc::proto::host_to_le16(0x0005);
	payload.speed_mm_s_le =
		(int16_t)mc::proto::host_to_le16((uint16_t)(int16_t)-123);
	payload.steer_cdeg_le =
		(int16_t)mc::proto::host_to_le16((uint16_t)(int16_t)456);
	payload.age_ms_le = mc::proto::host_to_le16(250);

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
	test_crc16();
	test_cobs_roundtrip();
	test_reader_ok();
	test_reader_crc_fail();
	test_reader_bad_length();
	test_reader_bad_version();
	test_golden_frame_output();
	test_writer_ack_roundtrip();
	test_writer_status_roundtrip();
	std::cout << "firmware proto tests ok\n";
	return 0;
}
