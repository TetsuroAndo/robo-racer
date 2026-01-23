#include <Arduino.h>
#include <unity.h>

#include <mc/proto/Proto.hpp>

static void test_crc16_vector() {
	const uint8_t data[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
	const uint16_t crc = mc::proto::crc16_ccitt(data, sizeof(data));
	TEST_ASSERT_EQUAL_HEX16(0x29B1, crc);
}

static void test_packet_roundtrip() {
	const uint8_t payload[2] = {0x01, 0x00};
	uint8_t encoded[mc::proto::MAX_FRAME_ENCODED];
	size_t out_len = 0;
	const bool ok = mc::proto::PacketWriter::build(
		encoded, sizeof(encoded), out_len, mc::proto::Type::PING, 0, 0x1234,
		payload, static_cast< uint16_t >(sizeof(payload)));
	TEST_ASSERT_TRUE(ok);

	mc::proto::PacketReader reader;
	for (size_t i = 0; i < out_len; ++i) {
		reader.push(encoded[i]);
	}
	TEST_ASSERT_TRUE(reader.hasFrame());

	const auto &frame = reader.frame();
	TEST_ASSERT_EQUAL_UINT8(mc::proto::VERSION, frame.ver());
	TEST_ASSERT_EQUAL_UINT8(static_cast< uint8_t >(mc::proto::Type::PING),
							frame.type());
	TEST_ASSERT_EQUAL_UINT16(0x1234, frame.seq());
	TEST_ASSERT_EQUAL_UINT16(sizeof(payload), frame.len());
	TEST_ASSERT_EQUAL_UINT16(sizeof(payload), frame.payload_len);
	TEST_ASSERT_EQUAL_UINT8_ARRAY(payload, frame.payload, sizeof(payload));
}

void setup() {
	UNITY_BEGIN();
	RUN_TEST(test_crc16_vector);
	RUN_TEST(test_packet_roundtrip);
	UNITY_END();
}

void loop() {}
