#include "config/Config.h"
#include "control/ControllerInput.h"
#include "hardware/Drive.h"
#include "hardware/ImuEstimator.h"
#include "hardware/Mpu6500.h"
#include "hardware/Tsd20.h"
#include <Arduino.h>
#include <cmath>

#include "../lib/common/Math.h"
#include "../lib/common/Time.h"
#include "comm/UartTx.h"
#include "comm/registry.h"
#include "log/AsyncLogger.h"
#include <mc/proto/Proto.hpp>

static mc::ControlState g_state;
static mc::Context g_ctx;

static Drive drive;
static Mpu6500 imu;
static ImuEstimator imu_est;
static Tsd20 tsd20;
static ControllerInput pad;
static mc::AsyncLogger alog;
static mc::UartTx uart_tx;

static TwoWire imu_wire(1);
static HardwareSerial tsd_uart(1);

static mc::proto::PacketReader reader;

static mc::PeriodicTimer statusTimer(50);
static mc::PeriodicTimer logTimer(200);
static mc::PeriodicTimer imuTimer(cfg::IMU_READ_INTERVAL_MS);
static mc::PeriodicTimer tsdTimer(cfg::TSD20_READ_INTERVAL_MS);
static mc::PeriodicTimer tsdInitTimer(cfg::TSD20_INIT_RETRY_MS);
static uint16_t status_seq = 0;
static uint16_t imu_seq = 0;

static bool g_tsd_ready = false;
static bool g_tsd_valid = false;
static uint16_t g_tsd_mm = 0;
static uint8_t g_tsd_fail_count = 0;
static bool g_tsd_fail_logged = false;
static uint32_t g_tsd_last_read_ms = 0;
static uint16_t g_tsd_period_ms = 0;

static bool g_imu_ready = false;
static bool g_imu_valid = false;
static uint32_t g_imu_last_ms = 0;
static ImuSample g_imu_sample{};
static int16_t g_last_cmd_speed_mm_s = 0;

static bool g_abs_active = false;
static uint32_t g_abs_cycle_ms = 0;
static float g_abs_duty = 0.0f;
static float g_abs_i = 0.0f;
static uint32_t g_abs_hold_until_ms = 0;

static inline void wr16(uint8_t *p, uint16_t v) {
	p[0] = (uint8_t)(v & 0xFF);
	p[1] = (uint8_t)(v >> 8);
}

#pragma pack(push, 1)
struct StatusPayload {
	uint8_t seq_applied;
	uint8_t auto_active;
	uint16_t faults_le;
	int16_t speed_mm_s_le;
	int16_t steer_cdeg_le;
	uint16_t age_ms_le;
};

struct ImuStatusPayload {
	int16_t a_long_mm_s2_le;
	int16_t v_est_mm_s_le;
	uint16_t a_brake_cap_mm_s2_le;
	int16_t yaw_dps_x10_le;
	uint16_t age_ms_le;
	uint8_t flags;
	uint8_t reserved;
};
#pragma pack(pop)

static void sendStatus_(uint32_t now_ms) {
	const bool auto_active = (g_state.mode == mc::Mode::AUTO);
	const bool hb_timeout = auto_active && (g_state.last_hb_ms != 0) &&
							((uint32_t)(now_ms - g_state.last_hb_ms) >
							 (uint32_t)cfg::HEARTBEAT_TIMEOUT_MS);
	const bool ttl_expired =
		auto_active && (g_state.cmd_expire_ms != 0) &&
		((uint32_t)now_ms > (uint32_t)g_state.cmd_expire_ms);
	const bool auto_inactive =
		(!auto_active) && (g_state.cmd_expire_ms != 0) &&
		((uint32_t)now_ms <= (uint32_t)g_state.cmd_expire_ms);

	uint16_t faults = 0;
	if (g_state.killed)
		faults |= 1u << 0; // KILL_LATCHED
	if (hb_timeout)
		faults |= 1u << 1; // HB_TIMEOUT
	if (ttl_expired)
		faults |= 1u << 2; // TTL_EXPIRED
	if (auto_inactive)
		faults |= 1u << 3; // AUTO_INACTIVE

	uint16_t age_ms = cfg::AUTO_CMD_AGE_UNKNOWN_MS;
	if (g_state.last_cmd_ms != 0) {
		uint32_t age = now_ms - g_state.last_cmd_ms;
		if (age > 0xFFFFu)
			age = 0xFFFFu;
		age_ms = (uint16_t)age;
	}

	StatusPayload p{};
	p.seq_applied = (uint8_t)(g_state.last_seq & 0xFFu);
	p.auto_active = auto_active ? 1u : 0u;
	wr16((uint8_t *)&p.faults_le, faults);
	wr16((uint8_t *)&p.speed_mm_s_le,
		 (uint16_t)(int16_t)drive.appliedSpeedMmS());
	wr16((uint8_t *)&p.steer_cdeg_le,
		 (uint16_t)(int16_t)drive.appliedSteerCdeg());
	wr16((uint8_t *)&p.age_ms_le, age_ms);

	uint8_t out[mc::proto::MAX_FRAME_ENCODED];
	size_t out_len = 0;
	mc::proto::PacketWriter::build(out, sizeof(out), out_len,
								   mc::proto::Type::STATUS, 0, status_seq++,
								   (const uint8_t *)&p, (uint16_t)sizeof(p));
	if (g_ctx.tx) {
		g_ctx.tx->enqueue(out, (uint16_t)out_len);
	}
}

static void sendImuStatus_(uint32_t now_ms) {
	if (!cfg::IMU_ENABLE)
		return;

	const ImuEstimate &st = imu_est.state();
	const uint16_t age_ms =
		g_imu_last_ms ? (uint16_t)mc::clamp< uint32_t >(now_ms - g_imu_last_ms,
														0u, 0xFFFFu)
					  : 0xFFFFu;

	ImuStatusPayload p{};
	wr16((uint8_t *)&p.a_long_mm_s2_le,
		 (uint16_t)(int16_t)lroundf(st.a_long_mm_s2));
	wr16((uint8_t *)&p.v_est_mm_s_le,
		 (uint16_t)(int16_t)lroundf(st.v_est_mm_s));
	wr16((uint8_t *)&p.a_brake_cap_mm_s2_le,
		 (uint16_t)mc::clamp< int >((int)lroundf(st.a_brake_cap_mm_s2), 0,
									0xFFFF));
	const int yaw_x10 = (int)lroundf(st.gz_dps * 10.0f);
	wr16((uint8_t *)&p.yaw_dps_x10_le,
		 (uint16_t)(int16_t)mc::clamp< int >(yaw_x10, -32768, 32767));
	wr16((uint8_t *)&p.age_ms_le, age_ms);
	uint8_t flags = 0;
	if (g_imu_valid)
		flags |= 1u << 0;
	if (st.calibrated)
		flags |= 1u << 1;
	if (g_abs_active)
		flags |= 1u << 2;
	p.flags = flags;
	p.reserved = 0;

	uint8_t out[mc::proto::MAX_FRAME_ENCODED];
	size_t out_len = 0;
	mc::proto::PacketWriter::build(out, sizeof(out), out_len,
								   mc::proto::Type::IMU_STATUS, 0, imu_seq++,
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

			mc::IHandler *h = mc::Registry::instance().get(f.type());
			if (h) {
				(void)h->onFrame(f, g_ctx, now_ms);
			} else if (g_ctx.log) {
				g_ctx.log->logf(mc::LogLevel::WARN, "proto",
								"RX unknown type=0x%02X", (unsigned)f.type());
			}
			reader.consumeFrame();
		}
	}
}

static int16_t clampSpeedWithTsd20_(int16_t speed_mm_s, mc::Mode mode,
									float a_brake_cap_mm_s2) {
	if (!cfg::TSD20_ENABLE)
		return speed_mm_s;
	if (speed_mm_s <= 0)
		return speed_mm_s;
	if (!cfg::TSD20_CLAMP_IN_MANUAL && mode != mc::Mode::AUTO)
		return speed_mm_s;
	if (cfg::TSD20_REQUIRE_OK && !g_tsd_ready)
		return 0;

	if (!g_tsd_valid) {
		if (g_tsd_fail_count >= cfg::TSD20_MAX_FAILS)
			return 0;
		return speed_mm_s;
	}

	if (g_tsd_mm <= cfg::TSD20_MARGIN_MM)
		return 0;

	const float a_min = (float)cfg::IMU_BRAKE_MIN_MM_S2;
	const float a_max = (float)cfg::IMU_BRAKE_MAX_MM_S2;
	float a = a_brake_cap_mm_s2;
	if (a <= 0.0f)
		a = (float)cfg::IMU_BRAKE_INIT_MM_S2;
	if (a < a_min)
		a = a_min;
	else if (a > a_max)
		a = a_max;

	const float d_allow = (float)g_tsd_mm - (float)cfg::TSD20_MARGIN_MM;
	const float tau = (float)cfg::TSD20_LATENCY_MS / 1000.0f;
	const float term = a * tau;
	const float disc = term * term + 2.0f * a * d_allow;
	float v_max = (disc > 0.0f) ? (-term + std::sqrt(disc)) : 0.0f;
	if (v_max < 0.0f)
		v_max = 0.0f;

	const float v_cap = std::min(v_max, (float)cfg::DRIVE_SPEED_MAX_MM_S);
	if ((float)speed_mm_s > v_cap)
		return (int16_t)std::lround(v_cap);
	return speed_mm_s;
}

static void resetAbs_(uint32_t now_ms) {
	g_abs_active = false;
	g_abs_cycle_ms = now_ms;
	g_abs_duty = 0.0f;
	g_abs_i = 0.0f;
	g_abs_hold_until_ms = 0;
}

static int16_t applyAbsBrake_(uint32_t now_ms, float dt_s, int16_t speed_mm_s,
							  bool allow_abs) {
	if (!cfg::ABS_ENABLE || !allow_abs) {
		resetAbs_(now_ms);
		return speed_mm_s;
	}

	if (!g_imu_valid) {
		resetAbs_(now_ms);
		return speed_mm_s;
	}

	const ImuEstimate &st = imu_est.state();
	if (!st.calibrated) {
		resetAbs_(now_ms);
		return speed_mm_s;
	}

	if (speed_mm_s < 0) {
		resetAbs_(now_ms);
		return speed_mm_s;
	}

	const float v_cmd = std::max(0.0f, (float)speed_mm_s);
	const float v_est = std::max(0.0f, st.v_est_mm_s);
	const float margin = (float)cfg::ABS_SPEED_MARGIN_MM_S;
	const bool want_brake = (v_est > (v_cmd + margin));

	if (want_brake) {
		g_abs_hold_until_ms = now_ms + cfg::ABS_BRAKE_HOLD_MS;
	}
	const bool active =
		want_brake || (g_abs_active && (g_abs_hold_until_ms != 0) &&
					   (now_ms <= g_abs_hold_until_ms));

	if (!active) {
		resetAbs_(now_ms);
		return speed_mm_s;
	}

	if (!g_abs_active) {
		g_abs_active = true;
		g_abs_cycle_ms = now_ms;
		g_abs_duty = (float)cfg::ABS_DUTY_MIN / 100.0f;
		g_abs_i = 0.0f;
	}

	float a_cap = st.a_brake_cap_mm_s2;
	if (a_cap <= 0.0f)
		a_cap = (float)cfg::IMU_BRAKE_INIT_MM_S2;
	if (a_cap < (float)cfg::IMU_BRAKE_MIN_MM_S2)
		a_cap = (float)cfg::IMU_BRAKE_MIN_MM_S2;
	if (a_cap > (float)cfg::IMU_BRAKE_MAX_MM_S2)
		a_cap = (float)cfg::IMU_BRAKE_MAX_MM_S2;

	float dv = v_est - v_cmd;
	if (dv < 0.0f)
		dv = 0.0f;
	float t_brake = (float)cfg::ABS_BRAKE_HOLD_MS / 1000.0f;
	if (t_brake < 0.05f)
		t_brake = 0.05f;
	float a_target = (dv / t_brake) * cfg::ABS_A_TARGET_SCALE;
	if (a_target > a_cap)
		a_target = a_cap;
	if (a_target < 0.0f)
		a_target = 0.0f;

	const float decel = std::max(0.0f, -st.a_long_mm_s2);
	const float e = a_target - decel;
	g_abs_i += e * cfg::ABS_DUTY_KI * dt_s;
	float duty = g_abs_duty + cfg::ABS_DUTY_KP * e + g_abs_i;

	const float duty_min = (float)cfg::ABS_DUTY_MIN / 100.0f;
	const float duty_max = (float)cfg::ABS_DUTY_MAX / 100.0f;
	duty = mc::clamp< float >(duty, duty_min, duty_max);
	g_abs_duty = duty;

	if (cfg::ABS_PERIOD_MS == 0) {
		return speed_mm_s;
	}

	const uint32_t period = cfg::ABS_PERIOD_MS;
	if ((uint32_t)(now_ms - g_abs_cycle_ms) >= period) {
		const uint32_t elapsed = (uint32_t)(now_ms - g_abs_cycle_ms);
		g_abs_cycle_ms = now_ms - (elapsed % period);
	}

	const uint32_t phase = (uint32_t)(now_ms - g_abs_cycle_ms);
	const uint32_t duty_ms = (uint32_t)lroundf((float)period * duty);
	const bool reverse_on = (phase < duty_ms);

	if (cfg::TSD20_ENABLE && g_tsd_valid) {
		const uint16_t limit = (uint16_t)(cfg::TSD20_MARGIN_MM +
										  cfg::ABS_REVERSE_DISABLE_MARGIN_MM);
		if (g_tsd_mm <= limit) {
			return 0;
		}
	}

	const int max_mm_s = cfg::DRIVE_SPEED_MAX_MM_S;
	int reverse_mm_s = cfg::ABS_REVERSE_MM_S;
	if (reverse_mm_s > max_mm_s)
		reverse_mm_s = max_mm_s;
	if (reverse_mm_s < 0)
		reverse_mm_s = -reverse_mm_s;

	return reverse_on ? (int16_t)(-reverse_mm_s) : 0;
}

static void updateTsd20_(uint32_t now_ms) {
	if (!cfg::TSD20_ENABLE)
		return;

	if (!g_tsd_ready && tsdInitTimer.due(now_ms)) {
		g_tsd_ready = tsd20.begin(tsd_uart);
		if (g_tsd_ready) {
			(void)tsd20.setLaser(true);
			g_tsd_fail_count = 0;
			g_tsd_fail_logged = false;
			alog.logf(mc::LogLevel::INFO, "tsd20",
					  "init ok id=0x%02X swapped=%d", (unsigned)tsd20.id(),
					  (int)tsd20.swapped());
		} else {
			alog.logf(mc::LogLevel::WARN, "tsd20", "init retry failed");
		}
	}

	if (!tsdTimer.due(now_ms))
		return;

	if (!g_tsd_ready) {
		g_tsd_valid = false;
		g_tsd_mm = 0;
		g_tsd_last_read_ms = 0;
		g_tsd_period_ms = 0;
		return;
	}

	uint16_t mm = 0;
	if (tsd20.readDistanceMm(mm)) {
		g_tsd_mm = mm;
		g_tsd_valid = true;
		g_tsd_fail_count = 0;
		g_tsd_fail_logged = false;
		if (g_tsd_last_read_ms != 0) {
			const uint32_t dt = now_ms - g_tsd_last_read_ms;
			g_tsd_period_ms = (uint16_t)mc::clamp< uint32_t >(dt, 0u, 0xFFFFu);
		}
		g_tsd_last_read_ms = now_ms;
	} else {
		if (g_tsd_fail_count < 0xFF)
			g_tsd_fail_count++;
		if (g_tsd_fail_count >= cfg::TSD20_MAX_FAILS) {
			g_tsd_valid = false;
			g_tsd_mm = 0;
			if (!g_tsd_fail_logged) {
				alog.logf(mc::LogLevel::WARN, "tsd20", "read failed (fails=%u)",
						  (unsigned)g_tsd_fail_count);
				g_tsd_fail_logged = true;
			}
		}
	}
}

static void applyTargets_(uint32_t now_ms, float dt_s) {
	const bool cmd_fresh =
		(g_state.cmd_expire_ms != 0) && (now_ms <= g_state.cmd_expire_ms);
	const bool abs_allowed =
		!g_state.killed &&
		((cfg::ABS_ENABLE_IN_MANUAL && g_state.mode == mc::Mode::MANUAL) ||
		 (g_state.mode == mc::Mode::AUTO));
	if (g_state.mode == mc::Mode::MANUAL) {
		pad.update();
		if (pad.isConnected()) {
			const PadState &st = pad.state();
			int forward = st.rt;
			int back = st.lt;
			int v = forward - back;
			int16_t speed_mm_s = (int16_t)mc::clamp< int >(
				v * 2, -cfg::DRIVE_SPEED_MAX_MM_S, cfg::DRIVE_SPEED_MAX_MM_S);
			speed_mm_s = clampSpeedWithTsd20_(
				speed_mm_s, g_state.mode, imu_est.state().a_brake_cap_mm_s2);

			int16_t steer = 0;
			if (st.dpad & DPAD_LEFT)
				steer = +1500;
			if (st.dpad & DPAD_RIGHT)
				steer = -1500;

			speed_mm_s = applyAbsBrake_(now_ms, dt_s, speed_mm_s, abs_allowed);
			g_last_cmd_speed_mm_s = speed_mm_s;
			drive.setBrakeMode(g_abs_active);
			drive.setTargetMmS(speed_mm_s);
			drive.setTargetSteerCdeg(steer);
			drive.setTtlMs(100);
			drive.setDistMm(0);
		} else {
			// AUTO_ACTIVE=false のときは UART setpoint を適用しない
			const int16_t speed_mm_s =
				applyAbsBrake_(now_ms, dt_s, 0, abs_allowed);
			g_last_cmd_speed_mm_s = speed_mm_s;
			drive.setBrakeMode(g_abs_active);
			drive.setTargetMmS(speed_mm_s);
			drive.setTargetSteerCdeg(0);
			drive.setTtlMs(100);
			drive.setDistMm(0);
		}
	} else {
		if (cmd_fresh) {
			int16_t speed_mm_s =
				clampSpeedWithTsd20_(g_state.target_speed_mm_s, g_state.mode,
									 imu_est.state().a_brake_cap_mm_s2);
			speed_mm_s = applyAbsBrake_(now_ms, dt_s, speed_mm_s, abs_allowed);
			g_last_cmd_speed_mm_s = speed_mm_s;
			drive.setBrakeMode(g_abs_active);
			drive.setTargetMmS(speed_mm_s);
			drive.setTargetSteerCdeg(g_state.target_steer_cdeg);
			drive.setTtlMs(g_state.target_ttl_ms);
			drive.setDistMm(g_state.target_dist_mm);
		} else {
			const int16_t speed_mm_s =
				applyAbsBrake_(now_ms, dt_s, 0, abs_allowed);
			g_last_cmd_speed_mm_s = speed_mm_s;
			drive.setBrakeMode(g_abs_active);
			drive.setTargetMmS(speed_mm_s);
			drive.setTargetSteerCdeg(0);
			drive.setTtlMs(100);
			drive.setDistMm(0);
		}
	}

	drive.tick(now_ms, dt_s, g_state.killed);
}

void setup() {
	Serial.begin(115200);
	delay(200);

	Serial2.begin(cfg::SERIAL_BAUD, SERIAL_8N1, cfg::SERIAL_RX_PIN,
				  cfg::SERIAL_TX_PIN);
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
	alog.logf(mc::LogLevel::INFO, "boot", "ESP32 up baud=%d",
			  (int)cfg::SERIAL_BAUD);

	if (cfg::IMU_ENABLE) {
		g_imu_ready = imu.begin(imu_wire);
		if (g_imu_ready) {
			imu_est.reset(millis());
			alog.logf(mc::LogLevel::INFO, "imu", "init ok id=0x%02X",
					  (unsigned)imu.id());
		} else {
			alog.logf(mc::LogLevel::WARN, "imu", "init failed");
		}
	}

	g_tsd_ready = tsd20.begin(tsd_uart);
	if (g_tsd_ready) {
		(void)tsd20.setLaser(true);
		alog.logf(mc::LogLevel::INFO, "tsd20",
				  "init ok id=0x%02X swapped=%d freq_ack=%d iic_ack=%d",
				  (unsigned)tsd20.id(), (int)tsd20.swapped(),
				  (int)tsd20.freqAck(), (int)tsd20.iicAck());
	} else {
		alog.logf(mc::LogLevel::WARN, "tsd20", "init failed");
	}

	uint32_t now = millis();
	statusTimer.reset(now);
	logTimer.reset(now);
	imuTimer.reset(now);
	tsdTimer.reset(now);
	tsdInitTimer.reset(now);
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
	updateTsd20_(now_ms);

	if (cfg::IMU_ENABLE && g_imu_ready && imuTimer.due(now_ms)) {
		ImuSample sample{};
		if (imu.readSample(sample)) {
			g_imu_sample = sample;
			g_imu_valid = true;
			g_imu_last_ms = now_ms;
			imu_est.update(sample, now_ms, drive.appliedSpeedMmS(),
						   g_last_cmd_speed_mm_s);
		} else {
			g_imu_valid = false;
		}
	}

	applyTargets_(now_ms, dt_s);

	if (statusTimer.due(now_ms)) {
		sendStatus_(now_ms);
		sendImuStatus_(now_ms);
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
		if (cfg::TSD20_ENABLE) {
			alog.logf(mc::LogLevel::INFO, "tsd20",
					  "ready=%d valid=%d mm=%u fails=%u period=%ums "
					  "ack(freq=%d iic=%d)",
					  (int)g_tsd_ready, (int)g_tsd_valid, (unsigned)g_tsd_mm,
					  (unsigned)g_tsd_fail_count, (unsigned)g_tsd_period_ms,
					  (int)tsd20.freqAck(), (int)tsd20.iicAck());
		}
		if (cfg::IMU_ENABLE) {
			const ImuEstimate &st = imu_est.state();
			const uint32_t age =
				g_imu_last_ms ? (uint32_t)(now_ms - g_imu_last_ms) : 0xFFFFu;
			alog.logf(mc::LogLevel::INFO, "imu",
					  "ready=%d valid=%d calib=%d age=%ums ax=%d ay=%d az=%d "
					  "gx=%d gy=%d gz=%d a_long=%.1f a_lpf=%.1f a_fusion=%.1f "
					  "v_est=%.1f a_brake=%.0f",
					  (int)g_imu_ready, (int)g_imu_valid, (int)st.calibrated,
					  (unsigned)age, (int)g_imu_sample.ax, (int)g_imu_sample.ay,
					  (int)g_imu_sample.az, (int)g_imu_sample.gx,
					  (int)g_imu_sample.gy, (int)g_imu_sample.gz,
					  (double)st.a_long_mm_s2, (double)st.a_long_lpf_mm_s2,
					  (double)st.a_long_fusion_mm_s2, (double)st.v_est_mm_s,
					  (double)st.a_brake_cap_mm_s2);
		}
	}

	delay(1);
}
