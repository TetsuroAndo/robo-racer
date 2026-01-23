#include <mc/proto/Proto.hpp>

#include <cassert>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <vector>

namespace {

size_t build_raw_frame(const mc::proto::Header &hdr, const uint8_t *payload,
					   size_t payload_len, uint8_t *out, size_t out_max) {
	const size_t frame_len = sizeof(mc::proto::Header) + payload_len;
	const size_t total_len = frame_len + 2;
	if (total_len > out_max)
		return 0;
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
	std::cout << "  " << label << " (" << len << "): ";
	for (size_t i = 0; i < len; ++i) {
		std::cout << std::hex << std::setw(2) << std::setfill('0')
				  << static_cast< unsigned int >(data[i]);
		if (i + 1 != len)
			std::cout << " ";
	}
	std::cout << std::dec << "\n";
}

} // namespace

/**
 * @brief
 *   CRC16-CCITT known vector. Ensures polynomial/init/bit order correctness.
 */
static void test_crc16() {
	std::cout << "[TEST] crc16\n";
	const uint8_t data[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
	const uint16_t crc = mc::proto::crc16_ccitt(data, sizeof(data));
	std::cout << "  expected=0x29b1 actual=0x" << std::hex << std::setw(4)
			  << std::setfill('0') << crc << std::dec << "\n";
	assert(crc == 0x29B1);
}

/**
 * @brief
 *   COBS roundtrip with zeros.
 */
static void test_cobs_roundtrip() {
	std::cout << "[TEST] cobs_roundtrip\n";
	const uint8_t input[] = {0x11, 0x00, 0x22, 0x33, 0x00, 0x44};
	uint8_t encoded[32];
	uint8_t decoded[32];

	const size_t enc_len =
		mc::proto::cobs_encode(input, sizeof(input), encoded, sizeof(encoded));
	const size_t dec_len =
		mc::proto::cobs_decode(encoded, enc_len, decoded, sizeof(decoded));
	print_bytes("input", input, sizeof(input));
	print_bytes("encoded", encoded, enc_len);
	print_bytes("decoded", decoded, dec_len);
	assert(dec_len == sizeof(input));
	assert(std::memcmp(input, decoded, sizeof(input)) == 0);
}

/**
 * @brief
 *   COBS encode overflow returns 0.
 */
static void test_cobs_encode_overflow() {
	std::cout << "[TEST] cobs_encode_overflow\n";
	const uint8_t input[] = {0x11, 0x22, 0x33, 0x44};
	uint8_t small_out[2];
	const size_t enc_len = mc::proto::cobs_encode(input, sizeof(input),
												  small_out, sizeof(small_out));
	std::cout << "  expect enc_len=0 actual=" << enc_len << "\n";
	assert(enc_len == 0);
}

/**
 * @brief
 *   PacketWriter rejects payload_len > MAX_PAYLOAD.
 */
static void test_writer_payload_too_large() {
	std::cout << "[TEST] writer_payload_too_large\n";
	std::vector< uint8_t > payload(mc::proto::MAX_PAYLOAD + 1, 0xAA);
	uint8_t out[mc::proto::MAX_FRAME_ENCODED];
	size_t out_len = 0;
	const bool ok = mc::proto::PacketWriter::build(
		out, sizeof(out), out_len, mc::proto::Type::LOG, 0, 1, payload.data(),
		(uint16_t)payload.size());
	std::cout << "  expect ok=false actual=" << (ok ? "true" : "false") << "\n";
	assert(!ok);
}

/**
 * @brief
 *   Minimum frame (len=0) is accepted by PacketReader.
 */
static void test_min_frame_len0() {
	std::cout << "[TEST] min_frame_len0\n";
	mc::proto::Header hdr{};
	hdr.magic[0] = mc::proto::MAGIC0;
	hdr.magic[1] = mc::proto::MAGIC1;
	hdr.ver = mc::proto::VERSION;
	hdr.type = static_cast< uint8_t >(mc::proto::Type::PING);
	hdr.flags = 0;
	hdr.seq_le = mc::proto::to_le16(0x0042);
	hdr.len_le = mc::proto::to_le16(0);

	uint8_t raw[mc::proto::MAX_FRAME_DECODED];
	const size_t raw_len = build_raw_frame(hdr, nullptr, 0, raw, sizeof(raw));
	const auto encoded = encode_frame(raw, raw_len);

	mc::proto::PacketReader reader;
	for (uint8_t b : encoded)
		reader.push(b);
	assert(reader.hasFrame());
	const auto &f = reader.frame();
	std::cout << "  expected len=0 actual len=" << f.payload_len << "\n";
	assert(f.payload_len == 0);
}

/**
 * @brief
 *   Maximum payload frame is accepted.
 */
static void test_max_payload() {
	std::cout << "[TEST] max_payload\n";
	std::vector< uint8_t > payload(mc::proto::MAX_PAYLOAD, 0xAB);

	mc::proto::Header hdr{};
	hdr.magic[0] = mc::proto::MAGIC0;
	hdr.magic[1] = mc::proto::MAGIC1;
	hdr.ver = mc::proto::VERSION;
	hdr.type = static_cast< uint8_t >(mc::proto::Type::LOG);
	hdr.flags = 0;
	hdr.seq_le = mc::proto::to_le16(0x0011);
	hdr.len_le = mc::proto::to_le16((uint16_t)payload.size());

	uint8_t raw[mc::proto::MAX_FRAME_DECODED];
	const size_t raw_len =
		build_raw_frame(hdr, payload.data(), payload.size(), raw, sizeof(raw));
	const auto encoded = encode_frame(raw, raw_len);

	mc::proto::PacketReader reader;
	for (uint8_t b : encoded)
		reader.push(b);
	assert(reader.hasFrame());
	const auto &f = reader.frame();
	std::cout << "  expected len=" << payload.size()
			  << " actual len=" << f.payload_len << "\n";
	assert(f.payload_len == payload.size());
}

/**
 * @brief
 *   Back-to-back frames decode without losing sync.
 */
static void test_back_to_back_frames() {
	std::cout << "[TEST] back_to_back_frames\n";
	std::vector< uint8_t > encoded_all;
	for (int i = 0; i < 2; ++i) {
		uint8_t payload[1] = {static_cast< uint8_t >(i)};
		mc::proto::Header hdr{};
		hdr.magic[0] = mc::proto::MAGIC0;
		hdr.magic[1] = mc::proto::MAGIC1;
		hdr.ver = mc::proto::VERSION;
		hdr.type = static_cast< uint8_t >(mc::proto::Type::PING);
		hdr.flags = 0;
		hdr.seq_le = mc::proto::to_le16((uint16_t)i);
		hdr.len_le = mc::proto::to_le16(1);
		uint8_t raw[mc::proto::MAX_FRAME_DECODED];
		const size_t raw_len =
			build_raw_frame(hdr, payload, sizeof(payload), raw, sizeof(raw));
		const auto enc = encode_frame(raw, raw_len);
		encoded_all.insert(encoded_all.end(), enc.begin(), enc.end());
	}

	mc::proto::PacketReader reader;
	int frames = 0;
	for (uint8_t b : encoded_all) {
		reader.push(b);
		if (reader.hasFrame()) {
			const auto &f = reader.frame();
			std::cout << "  frame " << frames << " seq=" << f.seq()
					  << " len=" << f.payload_len << "\n";
			frames++;
			reader.consumeFrame();
		}
	}
	std::cout << "  expected frames=2 actual=" << frames << "\n";
	assert(frames == 2);
}

/**
 * @brief
 *   Resync after a corrupted frame (CRC mismatch).
 */
static void test_resync_after_bad_crc() {
	std::cout << "[TEST] resync_after_bad_crc\n";
	uint8_t payload[1] = {0xAA};
	mc::proto::Header hdr{};
	hdr.magic[0] = mc::proto::MAGIC0;
	hdr.magic[1] = mc::proto::MAGIC1;
	hdr.ver = mc::proto::VERSION;
	hdr.type = static_cast< uint8_t >(mc::proto::Type::PING);
	hdr.flags = 0;
	hdr.seq_le = mc::proto::to_le16(0x10);
	hdr.len_le = mc::proto::to_le16(1);

	uint8_t raw[mc::proto::MAX_FRAME_DECODED];
	const size_t raw_len =
		build_raw_frame(hdr, payload, sizeof(payload), raw, sizeof(raw));
	raw[sizeof(mc::proto::Header)] ^= 0xFF; // corrupt payload
	const auto bad = encode_frame(raw, raw_len);

	// good frame afterwards
	hdr.seq_le = mc::proto::to_le16(0x11);
	const size_t raw_len2 =
		build_raw_frame(hdr, payload, sizeof(payload), raw, sizeof(raw));
	const auto good = encode_frame(raw, raw_len2);

	std::vector< uint8_t > stream;
	stream.insert(stream.end(), bad.begin(), bad.end());
	stream.insert(stream.end(), good.begin(), good.end());

	mc::proto::PacketReader reader;
	int ok_frames = 0;
	for (uint8_t b : stream) {
		reader.push(b);
		if (reader.hasFrame()) {
			const auto &f = reader.frame();
			std::cout << "  got frame seq=" << f.seq() << "\n";
			ok_frames++;
			reader.consumeFrame();
		}
	}
	std::cout << "  expected ok_frames=1 actual=" << ok_frames << "\n";
	assert(ok_frames == 1);
}

/**
 * @brief
 *   Header/length error cases: magic mismatch, bad version, len mismatch,
 *   len > MAX_PAYLOAD, and bad COBS.
 */
static void test_header_and_length_errors() {
	std::cout << "[TEST] header_and_length_errors\n";
	mc::proto::PacketReader reader;

	// bad magic
	mc::proto::Header hdr{};
	hdr.magic[0] = 'X';
	hdr.magic[1] = 'Y';
	hdr.ver = mc::proto::VERSION;
	hdr.type = static_cast< uint8_t >(mc::proto::Type::PING);
	hdr.flags = 0;
	hdr.seq_le = mc::proto::to_le16(0x20);
	hdr.len_le = mc::proto::to_le16(0);

	uint8_t raw[mc::proto::MAX_FRAME_DECODED];
	size_t raw_len = build_raw_frame(hdr, nullptr, 0, raw, sizeof(raw));
	auto enc = encode_frame(raw, raw_len);
	for (uint8_t b : enc)
		reader.push(b);
	std::cout << "  badHdr after bad magic=" << reader.badHdr() << "\n";
	assert(reader.badHdr() >= 1);

	// bad version
	hdr.magic[0] = mc::proto::MAGIC0;
	hdr.magic[1] = mc::proto::MAGIC1;
	hdr.ver = mc::proto::VERSION + 1;
	raw_len = build_raw_frame(hdr, nullptr, 0, raw, sizeof(raw));
	enc = encode_frame(raw, raw_len);
	for (uint8_t b : enc)
		reader.push(b);
	std::cout << "  badHdr after bad ver=" << reader.badHdr() << "\n";
	assert(reader.badHdr() >= 2);

	// len mismatch
	hdr.ver = mc::proto::VERSION;
	hdr.len_le = mc::proto::to_le16(2);
	uint8_t payload[1] = {0x01};
	raw_len = build_raw_frame(hdr, payload, sizeof(payload), raw, sizeof(raw));
	enc = encode_frame(raw, raw_len);
	for (uint8_t b : enc)
		reader.push(b);
	std::cout << "  badHdr after len mismatch=" << reader.badHdr() << "\n";
	assert(reader.badHdr() >= 3);

	// len > MAX_PAYLOAD
	hdr.len_le = mc::proto::to_le16(mc::proto::MAX_PAYLOAD + 1);
	raw_len = build_raw_frame(hdr, nullptr, 0, raw, sizeof(raw));
	enc = encode_frame(raw, raw_len);
	for (uint8_t b : enc)
		reader.push(b);
	std::cout << "  badHdr after len>MAX=" << reader.badHdr() << "\n";
	assert(reader.badHdr() >= 4);

	// Invalid COBS: code says 1 data byte follows, but frame ends immediately.
	const uint8_t bad_cobs[] = {0x02, 0x00};
	for (uint8_t b : bad_cobs)
		reader.push(b);
	std::cout << "  badCobs after bad cobs=" << reader.badCobs() << "\n";
	assert(reader.badCobs() >= 1);
}

int main() {
	test_crc16();
	test_cobs_roundtrip();
	test_cobs_encode_overflow();
	test_min_frame_len0();
	test_max_payload();
	test_back_to_back_frames();
	test_resync_after_bad_crc();
	test_header_and_length_errors();
	test_writer_payload_too_large();
	std::cout << "rpi proto tests ok\n";
	return 0;
}
