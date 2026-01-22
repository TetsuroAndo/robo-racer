#include "comm/Context.h"
#include "comm/Dispatcher.h"
#include "comm/TxPort.h"
#include "comm/handlers/Handlers.h"
#include "comm/protocol/PacketReader.h"
#include "comm/protocol/Protocol.h"
#include "config/Config.h"
#include "control/AutoCommandStore.h"
#include "control/ControllerInput.h"
#include "control/SafetyState.h"
#include "hardware/Drive.h"
#include "log/Logger.h"
#include <Arduino.h>

static ControllerInput pad;
static Drive drive;
static Logger logg;

static SafetyState safety;
static AutoCommandStore auto_cmd;
static TxPort tx;
static Dispatcher dispatcher;
static proto::PacketReader packet_reader;
static Context ctx{drive, safety, auto_cmd, tx};

static int manual_speed = 0;
static float manual_angle = 0.0f;
static int16_t status_speed_cmd = 0;
static int16_t status_steer_cdeg = 0;
static uint8_t status_seq = 0;
static uint32_t last_status_ms = 0;

namespace {
inline const proto::Header *hdr(const proto::FrameView &f) {
	return reinterpret_cast< const proto::Header * >(f.data);
}
} // namespace

void setup() {
	logg.begin(cfg::LOG_BAUD);

	Serial2.begin(cfg::SERIAL_BAUD, SERIAL_8N1, cfg::SERIAL_RX_PIN,
				  cfg::SERIAL_TX_PIN);
	tx.begin(Serial2);

	Serial.println("\n=== ESP32 UART Protocol (COBS+CRC) ===");

	registerHandlers(dispatcher);

	pad.begin();
	drive.begin();
}

static void uart_rx_poll() {
	while (Serial2.available() > 0) {
		const uint8_t byte = static_cast< uint8_t >(Serial2.read());
		proto::FrameView frame{};
		const auto res = packet_reader.push(byte, frame);
		if (res != proto::PacketReader::Result::kOk) {
			continue;
		}

		const auto *h = hdr(frame);
		if (h->ver != proto::VER) {
			if (h->flags & proto::FLAG_ACK_REQ) {
				tx.sendAck(h->type, h->seq, cfg::ACK_CODE_VERSION_MISMATCH);
			}
			continue;
		}

		if (!dispatcher.dispatch(h->type, frame, ctx)) {
			if (h->flags & proto::FLAG_ACK_REQ) {
				tx.sendAck(h->type, h->seq, cfg::ACK_CODE_UNHANDLED);
			}
		}
	}
}

static void bt_poll() {
	pad.update();

	if (!pad.isConnected()) {
		manual_speed = 0;
		manual_angle = 0.0f;
		return;
	}

	const PadState &st = pad.state();
	if (st.home || st.start) {
		safety.kill_latched = true;
	}

	if (safety.auto_active) {
		manual_speed = 0;
		manual_angle = 0.0f;
		return;
	}

	if (st.dpad & 0x04) {
		manual_angle = cfg::MANUAL_STEER_DEG;
	} else if (st.dpad & 0x08) {
		manual_angle = -cfg::MANUAL_STEER_DEG;
	} else {
		manual_angle = 0.0f;
	}

	if (st.B) {
		manual_speed -= cfg::MANUAL_SPEED_STEP;
	} else if (st.A) {
		manual_speed += cfg::MANUAL_SPEED_STEP;
	} else {
		manual_speed -= cfg::MANUAL_SPEED_STEP;
	}

	manual_speed = constrain(manual_speed, 0, cfg::MANUAL_SPEED_MAX);
}

static void maybe_send_status(uint32_t now_ms) {
	if ((uint32_t)(now_ms - last_status_ms) < cfg::STATUS_INTERVAL_MS) {
		return;
	}

	proto::StatusPayload payload{};
	payload.seq_applied = auto_cmd.applied_seq;
	payload.auto_active = safety.auto_active ? 1 : 0;
	payload.fault = safety.fault;
	payload.speed_now = status_speed_cmd;
	payload.steer_now_cdeg = status_steer_cdeg;
	payload.age_ms = auto_cmd.ageMs(now_ms);

	if (tx.sendStatus(payload, status_seq)) {
		last_status_ms = now_ms;
		status_seq = static_cast< uint8_t >(status_seq + 1);
		safety.consumeAutoInactive();
	}
}

static void apply_control() {
	const uint32_t now_ms = millis();
	const bool hb_timeout = safety.auto_active && safety.last_hb_ms != 0 &&
							(now_ms - safety.last_hb_ms > safety.hb_timeout_ms);
	const bool ttl_expired = safety.auto_active && auto_cmd.ttlExpired(now_ms);

	safety.updateFaults();
	if (hb_timeout) {
		safety.fault |= SafetyState::HB_TIMEOUT;
	}
	if (ttl_expired) {
		safety.fault |= SafetyState::TTL_EXPIRED;
	}
	if (safety.auto_inactive_seen) {
		safety.fault |= SafetyState::AUTO_INACTIVE;
	}

	int cmd_speed = 0;
	float cmd_angle = 0.0f;
	if (safety.kill_latched) {
		cmd_speed = 0;
		cmd_angle = 0.0f;
	} else if (safety.auto_active) {
		if (hb_timeout || ttl_expired) {
			cmd_speed = 0;
			cmd_angle = 0.0f;
		} else {
			cmd_speed = auto_cmd.sp.speed_cmd;
			cmd_angle = static_cast< float >(auto_cmd.sp.steer_cdeg) /
						cfg::STEER_CDEG_SCALE;
			auto_cmd.applied_seq = auto_cmd.seq;
		}
	} else {
		cmd_speed = manual_speed;
		cmd_angle = manual_angle;
	}

	status_speed_cmd = static_cast< int16_t >(cmd_speed);
	status_steer_cdeg =
		static_cast< int16_t >(cmd_angle * cfg::STEER_CDEG_SCALE);

	drive.setSpeed(cmd_speed);
	drive.setAngle(cmd_angle);

	drive.control();
	maybe_send_status(now_ms);
}

void loop() {
	uart_rx_poll();
	bt_poll();
	apply_control();
}
