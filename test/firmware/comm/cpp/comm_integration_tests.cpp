#include "comm/UartTx.h"
#include "comm/registry.h"
#include <mc/proto/Proto.hpp>

#include "log/AsyncLogger.h"

#include <cassert>
#include <cstring>
#include <iostream>
#include <vector>

namespace mc::test {
void reset_tx();
const std::vector< std::vector< uint8_t > > &frames();
} // namespace mc::test

namespace {

struct TestCtx {
	mc::ControlState st{};
	mc::Context ctx{};
	mc::UartTx tx{};
};

std::vector< uint8_t > build_frame(mc::proto::Type type, uint8_t flags,
								   uint16_t seq, const uint8_t *payload,
								   uint16_t payload_len) {
	uint8_t out[mc::proto::MAX_FRAME_ENCODED];
	size_t out_len = 0;
	const bool ok = mc::proto::PacketWriter::build(
		out, sizeof(out), out_len, type, flags, seq, payload, payload_len);
	assert(ok);
	return std::vector< uint8_t >(out, out + out_len);
}

mc::proto::FrameView decode_one(const std::vector< uint8_t > &enc) {
	mc::proto::PacketReader reader;
	for (uint8_t b : enc) {
		reader.push(b);
	}
	assert(reader.hasFrame());
	return reader.frame();
}

void expect_ack_seq(const std::vector< uint8_t > &enc, uint16_t seq) {
	mc::proto::PacketReader reader;
	for (uint8_t b : enc) {
		reader.push(b);
	}
	assert(reader.hasFrame());
	const auto &f = reader.frame();
	assert(f.type() == (uint8_t)mc::proto::Type::ACK);
	assert(f.payload_len == 0);
	assert(f.seq() == seq);
}

} // namespace

/**
 * @brief
 *   Firmware "seriald-equivalent" integration:
 *   - decode UART frame
 *   - dispatch to handler via Registry
 *   - verify ACK response enqueued
 */
static void test_ack_for_mode_set() {
	std::cout << "[TEST] firmware_comm_ack_mode_set\n";
	mc::test::reset_tx();
	TestCtx t{};
	t.ctx.st = &t.st;
	t.ctx.tx = &t.tx;
	t.ctx.log = nullptr;
	t.ctx.uart = nullptr;

	// mode=1 (AUTO)
	const uint8_t payload[1] = {1};
	const auto enc = build_frame(mc::proto::Type::MODE_SET,
								 mc::proto::FLAG_ACK_REQ, 0x10, payload, 1);
	const auto f = decode_one(enc);

	mc::IHandler *h = mc::Registry::instance().get(f.type());
	assert(h != nullptr);
	h->onFrame(f, t.ctx, 123);

	const auto &frames = mc::test::frames();
	std::cout << "\tTX frames=" << frames.size() << " (expect 1 ACK)\n";
	assert(frames.size() == 1);
	expect_ack_seq(frames[0], 0x10);
}

/**
 * @brief
 *   DRIVE handler updates state and returns ACK on ACK_REQ.
 */
static void test_ack_for_drive_and_state() {
	std::cout << "[TEST] firmware_comm_ack_drive\n";
	mc::test::reset_tx();
	TestCtx t{};
	t.ctx.st = &t.st;
	t.ctx.tx = &t.tx;
	t.ctx.log = nullptr;
	t.ctx.uart = nullptr;

	uint8_t payload[8] = {};
	// steer=100, speed=-200, ttl=100, dist=0
	payload[0] = 100 & 0xFF;
	payload[1] = (100 >> 8) & 0xFF;
	payload[2] = (uint16_t)(int16_t)-200 & 0xFF;
	payload[3] = (uint16_t)(int16_t)-200 >> 8;
	payload[4] = 100 & 0xFF;
	payload[5] = (100 >> 8) & 0xFF;
	payload[6] = 0;
	payload[7] = 0;

	const auto enc = build_frame(mc::proto::Type::DRIVE,
								 mc::proto::FLAG_ACK_REQ, 0x20, payload, 8);
	const auto f = decode_one(enc);

	mc::IHandler *h = mc::Registry::instance().get(f.type());
	assert(h != nullptr);
	h->onFrame(f, t.ctx, 1000);

	std::cout << "\tEXPECT last_seq=0x20 steer=100 speed=-200\n";
	std::cout << "\tACTUAL last_seq=0x" << std::hex << t.st.last_seq << std::dec
			  << " steer=" << t.st.target_steer_cdeg
			  << " speed=" << t.st.target_speed_mm_s << "\n";
	assert(t.st.last_seq == 0x20);
	assert(t.st.target_steer_cdeg == 100);
	assert(t.st.target_speed_mm_s == -200);

	const auto &frames = mc::test::frames();
	std::cout << "\tTX frames=" << frames.size() << " (expect 1 ACK)\n";
	assert(frames.size() == 1);
	expect_ack_seq(frames[0], 0x20);
}

/**
 * @brief
 *   KILL handler latches and returns ACK on ACK_REQ.
 */
static void test_ack_for_kill() {
	std::cout << "[TEST] firmware_comm_ack_kill\n";
	mc::test::reset_tx();
	TestCtx t{};
	t.ctx.st = &t.st;
	t.ctx.tx = &t.tx;
	t.ctx.log = nullptr;
	t.ctx.uart = nullptr;

	const auto enc = build_frame(mc::proto::Type::KILL, mc::proto::FLAG_ACK_REQ,
								 0x30, nullptr, 0);
	const auto f = decode_one(enc);

	mc::IHandler *h = mc::Registry::instance().get(f.type());
	assert(h != nullptr);
	h->onFrame(f, t.ctx, 0);

	std::cout << "\tEXPECT killed=true\n";
	std::cout << "\tACTUAL killed=" << (t.st.killed ? "true" : "false") << "\n";
	assert(t.st.killed == true);
	const auto &frames = mc::test::frames();
	std::cout << "\tTX frames=" << frames.size() << " (expect 1 ACK)\n";
	assert(frames.size() == 1);
	expect_ack_seq(frames[0], 0x30);
}

/**
 * @brief
 *   MODE_SET len=2 (reason付き) を許可し、mode反映されることを確認。
 */
static void test_mode_len2_reason() {
	std::cout << "[TEST] firmware_comm_mode_len2\n";
	mc::test::reset_tx();
	TestCtx t{};
	t.ctx.st = &t.st;
	t.ctx.tx = &t.tx;
	t.ctx.log = nullptr;

	uint8_t payload[2] = {0, 42};
	const auto enc =
		build_frame(mc::proto::Type::MODE_SET, 0, 0x40, payload, 2);
	const auto f = decode_one(enc);
	mc::IHandler *h = mc::Registry::instance().get(f.type());
	assert(h != nullptr);
	h->onFrame(f, t.ctx, 0);
	std::cout << "\tEXPECT mode=MANUAL(0) ACTUAL=" << (int)t.st.mode << "\n";
	assert((int)t.st.mode == 0);
}

/**
 * @brief
 *   MODE_SET invalid mode returns error and does not ACK.
 */
static void test_mode_invalid_value() {
	std::cout << "[TEST] firmware_comm_mode_invalid_value\n";
	mc::test::reset_tx();
	TestCtx t{};
	t.ctx.st = &t.st;
	t.ctx.tx = &t.tx;
	t.ctx.log = nullptr;

	uint8_t payload[1] = {2};
	const auto enc = build_frame(mc::proto::Type::MODE_SET,
								 mc::proto::FLAG_ACK_REQ, 0x41, payload, 1);
	const auto f = decode_one(enc);
	mc::IHandler *h = mc::Registry::instance().get(f.type());
	assert(h != nullptr);
	h->onFrame(f, t.ctx, 0);

	const auto &frames = mc::test::frames();
	std::cout << "\tEXPECT no ACK frames, actual=" << frames.size() << "\n";
	assert(frames.empty());
}

/**
 * @brief
 *   PING len=0 は ACK が返り、last_hb_ms が更新されることを確認。
 *   len!=0 は不正として無視され、ACK も last_hb_ms 更新もされないことを確認。
 */
static void test_ping_len0_len4_ack() {
	std::cout << "[TEST] firmware_comm_ping_len0_len4\n";
	mc::test::reset_tx();
	TestCtx t{};
	t.ctx.st = &t.st;
	t.ctx.tx = &t.tx;
	t.ctx.log = nullptr;

	// len=0
	const auto enc0 = build_frame(mc::proto::Type::PING, 0, 0x50, nullptr, 0);
	const auto f0 = decode_one(enc0);
	mc::IHandler *h = mc::Registry::instance().get(f0.type());
	assert(h != nullptr);
	h->onFrame(f0, t.ctx, 100);
	std::cout << "\tlen=0 last_hb_ms=" << t.st.last_hb_ms << "\n";
	assert(t.st.last_hb_ms == 100);
	assert(mc::test::frames().size() == 1);
	expect_ack_seq(mc::test::frames()[0], 0x50);

	// len=4
	mc::test::reset_tx();
	uint8_t payload[4] = {1, 2, 3, 4};
	const auto enc4 = build_frame(mc::proto::Type::PING, 0, 0x51, payload, 4);
	const auto f4 = decode_one(enc4);
	h->onFrame(f4, t.ctx, 200);
	std::cout << "\tlen=4 last_hb_ms=" << t.st.last_hb_ms << "\n";
	assert(t.st.last_hb_ms == 100);
	assert(mc::test::frames().empty());
}

/**
 * @brief
 *   DRIVE len!=8 はエラーで ACK されない。
 */
static void test_drive_len_invalid() {
	std::cout << "[TEST] firmware_comm_drive_len_invalid\n";
	mc::test::reset_tx();
	TestCtx t{};
	t.ctx.st = &t.st;
	t.ctx.tx = &t.tx;
	t.ctx.log = nullptr;

	uint8_t payload[4] = {0};
	const auto enc = build_frame(mc::proto::Type::DRIVE,
								 mc::proto::FLAG_ACK_REQ, 0x60, payload, 4);
	const auto f = decode_one(enc);
	mc::IHandler *h = mc::Registry::instance().get(f.type());
	assert(h != nullptr);
	h->onFrame(f, t.ctx, 0);
	std::cout << "\tEXPECT no ACK, actual=" << mc::test::frames().size()
			  << "\n";
	assert(mc::test::frames().empty());
}

/**
 * @brief
 *   DRIVE clamp: steer/speed/ttl are clamped to min/max.
 */
static void test_drive_clamp() {
	std::cout << "[TEST] firmware_comm_drive_clamp\n";
	mc::test::reset_tx();
	TestCtx t{};
	t.ctx.st = &t.st;
	t.ctx.tx = &t.tx;
	t.ctx.log = nullptr;

	uint8_t payload[8] = {};
	// steer=5000 (clamp to 2500), speed=-9999 (clamp to -5000),
	// ttl=1 (clamp to 10), dist=0
	payload[0] = 0x88;
	payload[1] = 0x13; // 5000
	payload[2] = 0xF1;
	payload[3] = 0xD8; // -9999
	payload[4] = 0x01;
	payload[5] = 0x00; // ttl=1
	payload[6] = 0;
	payload[7] = 0;

	const auto enc = build_frame(mc::proto::Type::DRIVE, 0, 0x61, payload, 8);
	const auto f = decode_one(enc);
	mc::IHandler *h = mc::Registry::instance().get(f.type());
	assert(h != nullptr);
	h->onFrame(f, t.ctx, 1000);

	std::cout << "\tEXPECT steer=2500 speed=-5000 ttl=10\n";
	std::cout << "\tACTUAL steer=" << t.st.target_steer_cdeg
			  << " speed=" << t.st.target_speed_mm_s
			  << " ttl=" << t.st.target_ttl_ms << "\n";
	assert(t.st.target_steer_cdeg == 2500);
	assert(t.st.target_speed_mm_s == -5000);
	assert(t.st.target_ttl_ms == 10);
}

/**
 * @brief
 *   KILL len=2 でも動作する（payloadは無視）。
 */
static void test_kill_len2() {
	std::cout << "[TEST] firmware_comm_kill_len2\n";
	mc::test::reset_tx();
	TestCtx t{};
	t.ctx.st = &t.st;
	t.ctx.tx = &t.tx;
	t.ctx.log = nullptr;

	uint8_t payload[2] = {0, 1};
	const auto enc = build_frame(mc::proto::Type::KILL, 0, 0x70, payload, 2);
	const auto f = decode_one(enc);
	mc::IHandler *h = mc::Registry::instance().get(f.type());
	assert(h != nullptr);
	h->onFrame(f, t.ctx, 0);
	std::cout << "\tEXPECT killed=true ACTUAL="
			  << (t.st.killed ? "true" : "false") << "\n";
	assert(t.st.killed == true);
}

/**
 * @brief
 *   Unknown type should not crash (Registry returns null).
 */
static void test_unknown_type_no_crash() {
	std::cout << "[TEST] firmware_comm_unknown_type\n";
	mc::IHandler *h = mc::Registry::instance().get(0x99);
	std::cout << "\tEXPECT nullptr ACTUAL=" << (h ? "non-null" : "null")
			  << "\n";
	assert(h == nullptr);
}

int main() {
	test_ack_for_mode_set();
	test_ack_for_drive_and_state();
	test_ack_for_kill();
	test_mode_len2_reason();
	test_mode_invalid_value();
	test_ping_len0_len4_ack();
	test_drive_len_invalid();
	test_drive_clamp();
	test_kill_len2();
	test_unknown_type_no_crash();
	std::cout << "firmware comm integration tests ok\n";
	return 0;
}
