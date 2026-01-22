#include "proto/Codec.h"
#include "proto/PacketReader.h"
#include "proto/Protocol.h"

#include <cassert>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <vector>

namespace {

size_t build_raw_frame(const proto::Header &hdr, const uint8_t *payload,
					   size_t payload_len, uint8_t *out, size_t out_max) {
	const size_t frame_len = sizeof(proto::Header) + payload_len;
	const size_t total_len = frame_len + proto::CRC_SIZE;
	if (total_len > out_max) {
		return 0;
	}
	std::memcpy(out, &hdr, sizeof(proto::Header));
	if (payload_len > 0 && payload != nullptr) {
		std::memcpy(out + sizeof(proto::Header), payload, payload_len);
	}
	const uint16_t crc = proto::crc16_ccitt(out, frame_len);
	out[frame_len] = static_cast< uint8_t >(crc & 0xFF);
	out[frame_len + 1] = static_cast< uint8_t >((crc >> 8) & 0xFF);
	return total_len;
}

std::vector< uint8_t > encode_frame(const uint8_t *raw, size_t raw_len) {
	uint8_t cobs_buf[proto::MAX_ENCODED];
	const size_t encoded_len =
		proto::cobs_encode(raw, raw_len, cobs_buf, sizeof(cobs_buf));
	assert(encoded_len > 0);
	std::vector< uint8_t > out(cobs_buf, cobs_buf + encoded_len);
	out.push_back(0x00);
	return out;
}

void print_bytes(const char *label, const uint8_t *data, size_t len) {
	std::cout << "  " << label << " (" << len << "): ";
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
	const uint16_t crc = proto::crc16_ccitt(data, sizeof(data));
	std::cout << "  got: 0x" << std::hex << std::setw(4) << std::setfill('0')
			  << crc << std::dec << "\n";
	std::cout << "  expected: 0x29b1\n";
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
		proto::cobs_encode(input, sizeof(input), encoded, sizeof(encoded));
	assert(enc_len > 0);
	const size_t dec_len =
		proto::cobs_decode(encoded, enc_len, decoded, sizeof(decoded));
	print_bytes("input", input, sizeof(input));
	print_bytes("encoded", encoded, enc_len);
	print_bytes("decoded", decoded, dec_len);
	assert(dec_len == sizeof(input));
	assert(std::memcmp(input, decoded, sizeof(input)) == 0);
}

/**
 * @brief 正常なフレームを受信したときにPacketReaderがOKを返すことを検証する。
 *
 * Header/CRCを含むフレームをCOBS化して投入し、復元されたHeaderが
 * 送信元と一致することを確認する。
 */
static void test_reader_ok() {
	std::cout << "[TEST] packet_reader_ok\n";
	proto::Header hdr{};
	hdr.ver = proto::VER;
	hdr.type = static_cast< uint8_t >(proto::Type::AUTO_MODE);
	hdr.flags = 0;
	hdr.seq = 0x42;
	hdr.len = 2;
	const uint8_t payload[2] = {0x01, 0x00};

	uint8_t raw[proto::MAX_FRAME];
	const size_t raw_len =
		build_raw_frame(hdr, payload, sizeof(payload), raw, sizeof(raw));
	assert(raw_len > 0);

	const auto encoded = encode_frame(raw, raw_len);
	proto::PacketReader reader;
	proto::FrameView frame{};
	auto res = proto::PacketReader::Result::NONE;
	for (uint8_t b : encoded) {
		res = reader.push(b, frame);
	}
	std::cout << "  reader result: "
			  << (res == proto::PacketReader::Result::OK ? "OK" : "OTHER")
			  << "\n";
	const auto *out_hdr = reinterpret_cast< const proto::Header * >(frame.data);
	std::cout << "  header got: ver=" << static_cast< int >(out_hdr->ver)
			  << " type=0x" << std::hex << static_cast< int >(out_hdr->type)
			  << " flags=0x" << static_cast< int >(out_hdr->flags) << " seq=0x"
			  << static_cast< int >(out_hdr->seq) << std::dec
			  << " len=" << out_hdr->len << "\n";
	std::cout << "  header expected: ver=" << static_cast< int >(hdr.ver)
			  << " type=0x" << std::hex << static_cast< int >(hdr.type)
			  << " flags=0x" << static_cast< int >(hdr.flags) << " seq=0x"
			  << static_cast< int >(hdr.seq) << std::dec << " len=" << hdr.len
			  << "\n";
	assert(res == proto::PacketReader::Result::OK);
	assert(out_hdr->ver == proto::VER);
	assert(out_hdr->type == hdr.type);
	assert(out_hdr->seq == hdr.seq);
	assert(out_hdr->len == hdr.len);
}

/**
 * @brief CRCが壊れているフレームをPacketReaderが検出することを検証する。
 *
 * 1バイトを反転させたフレームを送信し、ResultがERRORになり、
 * lastErrorがCRC_MISMATCHになることを確認する。
 */
static void test_reader_crc_fail() {
	std::cout << "[TEST] packet_reader_crc_fail\n";
	proto::Header hdr{};
	hdr.ver = proto::VER;
	hdr.type = static_cast< uint8_t >(proto::Type::AUTO_SETPOINT);
	hdr.flags = 0;
	hdr.seq = 0x10;
	hdr.len = 1;
	uint8_t payload[1] = {0x7A};

	uint8_t raw[proto::MAX_FRAME];
	const size_t raw_len =
		build_raw_frame(hdr, payload, sizeof(payload), raw, sizeof(raw));
	assert(raw_len > 0);

	raw[sizeof(proto::Header)] ^= 0xFF;
	const auto encoded = encode_frame(raw, raw_len);

	proto::PacketReader reader;
	proto::FrameView frame{};
	auto res = proto::PacketReader::Result::NONE;
	for (uint8_t b : encoded) {
		res = reader.push(b, frame);
		if (res == proto::PacketReader::Result::ERROR) {
			break;
		}
	}
	std::cout << "  reader result: "
			  << (res == proto::PacketReader::Result::ERROR ? "ERROR" : "OTHER")
			  << "\n";
	std::cout << "  error got: "
			  << (reader.lastError() == proto::PacketReader::Error::CRC_MISMATCH
					  ? "CRC_MISMATCH"
					  : "OTHER")
			  << "\n";
	std::cout << "  error expected: CRC_MISMATCH\n";
	assert(res == proto::PacketReader::Result::ERROR);
	assert(reader.lastError() == proto::PacketReader::Error::CRC_MISMATCH);
}

/**
 * @brief ヘッダのlenと実際のpayload長が不一致のフレームを検出する。
 *
 * lenを意図的に不正値にし、ResultがERRORかつBAD_LENGTHになることを確認する。
 */
static void test_reader_bad_length() {
	std::cout << "[TEST] packet_reader_bad_length\n";
	proto::Header hdr{};
	hdr.ver = proto::VER;
	hdr.type = static_cast< uint8_t >(proto::Type::AUTO_MODE);
	hdr.flags = 0;
	hdr.seq = 0x20;
	hdr.len = 3;
	const uint8_t payload[2] = {0x00, 0x00};

	uint8_t raw[proto::MAX_FRAME];
	const size_t raw_len =
		build_raw_frame(hdr, payload, sizeof(payload), raw, sizeof(raw));
	assert(raw_len > 0);
	const auto encoded = encode_frame(raw, raw_len);

	proto::PacketReader reader;
	proto::FrameView frame{};
	auto res = proto::PacketReader::Result::NONE;
	for (uint8_t b : encoded) {
		res = reader.push(b, frame);
		if (res == proto::PacketReader::Result::ERROR) {
			break;
		}
	}
	std::cout << "  reader result: "
			  << (res == proto::PacketReader::Result::ERROR ? "ERROR" : "OTHER")
			  << "\n";
	std::cout << "  error got: "
			  << (reader.lastError() == proto::PacketReader::Error::BAD_LENGTH
					  ? "BAD_LENGTH"
					  : "OTHER")
			  << "\n";
	std::cout << "  error expected: BAD_LENGTH\n";
	assert(res == proto::PacketReader::Result::ERROR);
	assert(reader.lastError() == proto::PacketReader::Error::BAD_LENGTH);
}

/**
 * @brief プロトコルバージョンが異なるフレームを受信できることを確認する。
 *
 * PacketReader自体はフレームとして受理（OK）するが、
 * Header上のverが期待値ではないことを確認する。
 */
static void test_reader_bad_version() {
	std::cout << "[TEST] packet_reader_bad_version\n";
	proto::Header hdr{};
	hdr.ver = static_cast< uint8_t >(proto::VER + 1);
	hdr.type = static_cast< uint8_t >(proto::Type::HEARTBEAT);
	hdr.flags = 0x01;
	hdr.seq = 0x33;
	hdr.len = 0;

	uint8_t raw[proto::MAX_FRAME];
	const size_t raw_len = build_raw_frame(hdr, nullptr, 0, raw, sizeof(raw));
	assert(raw_len > 0);
	const auto encoded = encode_frame(raw, raw_len);

	proto::PacketReader reader;
	proto::FrameView frame{};
	auto res = proto::PacketReader::Result::NONE;
	for (uint8_t b : encoded) {
		res = reader.push(b, frame);
	}
	std::cout << "  reader result: "
			  << (res == proto::PacketReader::Result::OK ? "OK" : "OTHER")
			  << "\n";
	assert(res == proto::PacketReader::Result::OK);
	const auto *out_hdr = reinterpret_cast< const proto::Header * >(frame.data);
	std::cout << "  header got: ver=" << static_cast< int >(out_hdr->ver)
			  << " expected not " << static_cast< int >(proto::VER) << "\n";
	assert(out_hdr->ver != proto::VER);
}

/**
 * @brief CRC/フレーム生成の最終バイト列が期待値と一致することを確認する。
 *
 * 固定のHeader/ペイロードから生成されるrawフレームが
 * 既知のゴールデンバイト列と一致することを検証する。
 */
static void test_golden_frame_output() {
	std::cout << "[TEST] golden_frame_output\n";
	proto::Header hdr{};
	hdr.ver = 0x01;
	hdr.type = 0x81;
	hdr.flags = 0x01;
	hdr.seq = 0x10;
	hdr.len = 0x0004;
	const uint8_t payload[4] = {0xAA, 0xBB, 0xCC, 0xDD};

	uint8_t raw[proto::MAX_FRAME];
	const size_t raw_len =
		build_raw_frame(hdr, payload, sizeof(payload), raw, sizeof(raw));

	const uint8_t expected[] = {0x01, 0x81, 0x01, 0x10, 0x04, 0x00,
								0xAA, 0xBB, 0xCC, 0xDD, 0xDE, 0x69};
	print_bytes("raw", raw, raw_len);
	print_bytes("expected", expected, sizeof(expected));
	assert(raw_len == sizeof(expected));
	assert(std::memcmp(raw, expected, sizeof(expected)) == 0);
}

int main() {
	test_crc16();
	test_cobs_roundtrip();
	test_reader_ok();
	test_reader_crc_fail();
	test_reader_bad_length();
	test_reader_bad_version();
	test_golden_frame_output();
	std::cout << "proto tests ok\n";
	return 0;
}
