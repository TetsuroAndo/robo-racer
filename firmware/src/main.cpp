#include "control/ControllerInput.h"
#include "hardware/Drive.h"
#include <Arduino.h>

#include "../lib/common/Math.h"
#include "../lib/common/Time.h"
#include "comm/UartTx.h"
#include "comm/mc_proto.h"
#include "comm/registry.h"
#include "log/AsyncLogger.h"

static constexpr int SERIAL_RXD = 16;
static constexpr int SERIAL_TXD = 17;
static constexpr int DEFAULT_ESP_BAUD = 921600;

static mc::ControlState g_state;
static mc::Context g_ctx;

static Drive drive;
static ControllerInput pad;
static mc::AsyncLogger alog;
static mc::UartTx uart_tx;

static mc::proto::PacketReader reader;

static mc::PeriodicTimer statusTimer(50);
static mc::PeriodicTimer logTimer(200);

static inline void wr16(uint8_t *p, uint16_t v) {
	p[0] = (uint8_t)(v & 0xFF);
	p[1] = (uint8_t)(v >> 8);
}

#pragma pack(push, 1)
struct StatusPayload {
	uint16_t last_seq_le;
	int16_t speed_mm_s_le;
	int16_t steer_cdeg_le;
	uint16_t ttl_ms_le;
	uint16_t dist_mm_le;
	uint16_t faults_le;
	uint16_t rx_bad_crc_le;
	uint16_t rx_bad_cobs_le;
};
#pragma pack(pop)

static void sendStatus_(uint32_t now_ms) {
	(void)now_ms;
	StatusPayload p{};
	wr16((uint8_t *)&p.last_seq_le, (uint16_t)g_state.last_seq);
	wr16((uint8_t *)&p.speed_mm_s_le, (uint16_t)drive.appliedSpeedMmS());
	wr16((uint8_t *)&p.steer_cdeg_le, (uint16_t)drive.appliedSteerCdeg());
	wr16((uint8_t *)&p.ttl_ms_le, drive.ttlMs());
	wr16((uint8_t *)&p.dist_mm_le, drive.distMm());
	wr16((uint8_t *)&p.faults_le, g_state.killed ? 0x0001 : 0x0000);
	wr16((uint8_t *)&p.rx_bad_crc_le,
		 (uint16_t)mc::clamp< uint32_t >(reader.badCrc(), 0, 65535));
	wr16((uint8_t *)&p.rx_bad_cobs_le,
		 (uint16_t)mc::clamp< uint32_t >(reader.badCobs(), 0, 65535));

	uint8_t out[mc::proto::MAX_FRAME_ENCODED];
	size_t out_len = 0;
	mc::proto::PacketWriter::build(out, sizeof(out), out_len,
								   mc::proto::Type::STATUS, 0, 0,
								   (const uint8_t *)&p, (uint16_t)sizeof(p));
	if (g_ctx.tx) {
		g_ctx.tx->enqueue(out, (uint16_t)out_len);
	}
}

static void handleRx_(uint32_t now_ms) {
	while (Serial2.available() > 0) {
		uint8_t b = (uint8_t)Serial2.read();
		if (reader.push(b) && reader.hasFrame()) {
			const auto &f = reader.frame();

			mc::IHandler *h = mc::Registry::instance().get(f.hdr.type);
			if (h) {
				(void)h->onFrame(f, g_ctx, now_ms);
			} else if (g_ctx.log) {
				g_ctx.log->logf(mc::LogLevel::WARN, "proto",
								"RX unknown type=0x%02X", (unsigned)f.hdr.type);
			}
			reader.consumeFrame();
		}
	}
}

static void applyTargets_(uint32_t now_ms, float dt_s) {
	if (g_state.mode == mc::Mode::MANUAL) {
		pad.update();
		if (pad.isConnected()) {
			const PadState &st = pad.state();
			int forward = st.rt;
			int back = st.lt;
			int v = forward - back;
			int16_t speed_mm_s = (int16_t)mc::clamp< int >(v * 2, -2000, 2000);

			int16_t steer = 0;
			if (st.dpad & DPAD_LEFT)
				steer = +1500;
			if (st.dpad & DPAD_RIGHT)
				steer = -1500;

			drive.setTargetMmS(speed_mm_s);
			drive.setTargetSteerCdeg(steer);
			drive.setTtlMs(100);
			drive.setDistMm(0);
		} else {
			drive.setTargetMmS(g_state.target_speed_mm_s);
			drive.setTargetSteerCdeg(g_state.target_steer_cdeg);
			drive.setTtlMs(g_state.target_ttl_ms);
			drive.setDistMm(g_state.target_dist_mm);
		}
	} else {
		drive.setTargetMmS(g_state.target_speed_mm_s);
		drive.setTargetSteerCdeg(g_state.target_steer_cdeg);
		drive.setTtlMs(g_state.target_ttl_ms);
		drive.setDistMm(g_state.target_dist_mm);
	}

	drive.tick(now_ms, dt_s, g_state.killed);
}

void setup() {
	Serial.begin(115200);
	delay(200);

	Serial2.begin(DEFAULT_ESP_BAUD, SERIAL_8N1, SERIAL_RXD, SERIAL_TXD);
	Serial2.setRxBufferSize(4096);
	Serial2.setTxBufferSize(4096);

	g_ctx.st = &g_state;
	g_ctx.uart = &Serial2;
	g_ctx.log = &alog;
	g_ctx.tx = &uart_tx;

	pad.begin();
	drive.begin();

	uart_tx.begin(Serial2);
	alog.begin(uart_tx);
	alog.setMinLevel(mc::LogLevel::INFO);
	alog.logf(mc::LogLevel::INFO, "boot", "ESP32 up baud=%d", DEFAULT_ESP_BAUD);

	uint32_t now = millis();
	statusTimer.reset(now);
	logTimer.reset(now);
}

void loop() {
	static uint64_t last_us = mc::Time::us();
	uint64_t now_us = mc::Time::us();
	float dt_s = (float)(now_us - last_us) / 1e6f;
	if (dt_s < 0.0f)
		dt_s = 0.0f;
	if (dt_s > 0.05f)
		dt_s = 0.05f;
	last_us = now_us;

	uint32_t now_ms = (uint32_t)millis();

	handleRx_(now_ms);
	applyTargets_(now_ms, dt_s);

	if (statusTimer.due(now_ms)) {
		sendStatus_(now_ms);
	}

	if (logTimer.due(now_ms)) {
		const char *mode =
			(g_state.mode == mc::Mode::MANUAL) ? "MANUAL" : "AUTO";
		alog.logf(mc::LogLevel::INFO, "drive",
				  "mode=%s killed=%d applied(speed=%dmm/s steer=%dcdeg "
				  "ttl=%ums dist=%umm) drop=%u",
				  mode, (int)g_state.killed, (int)drive.appliedSpeedMmS(),
				  (int)drive.appliedSteerCdeg(), (unsigned)drive.ttlMs(),
				  (unsigned)drive.distMm(), (unsigned)alog.dropped());
	}

	delay(1);
}
