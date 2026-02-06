#include "config/Config.h"
#include "control/AutoCommandSource.h"
#include "control/SafetySupervisor.h"
#include "control/SpeedController.h"
#include "control/Targets.h"
#include "hardware/Drive.h"
#include "hardware/ImuEstimator.h"
#include "hardware/Mpu6500.h"
#include "hardware/Tsd20.h"
#include <Arduino.h>
#include <algorithm>
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
static AutoCommandSource cmd_source;
static SafetySupervisor safety;
static SpeedController speed_ctl;
static mc::AsyncLogger alog;
static mc::UartTx uart_tx;

static TwoWire imu_wire(1);
static HardwareSerial tsd_uart(1);

static mc::proto::PacketReader reader;

static mc::PeriodicTimer statusTimer(cfg::STATUS_INTERVAL_MS);
static mc::PeriodicTimer logTimer(200);
static mc::PeriodicTimer imuTimer(cfg::IMU_READ_INTERVAL_MS);
static mc::PeriodicTimer tsdTimer(cfg::TSD20_READ_INTERVAL_MS);
static mc::PeriodicTimer tsdInitTimer(cfg::TSD20_INIT_RETRY_MS);
static uint16_t status_seq = 0;
static uint16_t imu_seq = 0;
static uint16_t tsd_seq = 0;

static Tsd20State g_tsd_state{};
static bool g_tsd_fail_logged = false;
static uint32_t g_tsd_last_read_ms = 0;
static uint16_t g_tsd_period_ms = 0;

static bool g_imu_ready = false;
static bool g_imu_valid = false;
static uint32_t g_imu_last_ms = 0;
static ImuSample g_imu_sample{};
static int16_t g_last_cmd_speed_mm_s = 0;

static bool g_abs_active = false;
static float g_last_dt_s = 0.0f;

static SafetyDiag g_safety_diag{};

static float g_speed_diag_v_cmd = 0.0f;
static float g_speed_diag_v_est = 0.0f;
static float g_speed_diag_err = 0.0f;
static float g_speed_diag_i = 0.0f;
static int16_t g_speed_diag_pwm_cmd = 0;
static int16_t g_speed_diag_pwm_ff = 0;
static bool g_speed_diag_saturated = false;
static bool g_speed_diag_active = false;
static bool g_speed_diag_calib = false;

static inline void wr16(uint8_t *p, uint16_t v) {
	p[0] = (uint8_t)(v & 0xFF);
	p[1] = (uint8_t)(v >> 8);
}

// shared/proto のワイヤ構造体をそのまま利用し、サイズ一致をここでも検証する。
static_assert(sizeof(mc::proto::StatusPayload) == 10,
			  "StatusPayload size (wire) must be 10 bytes");
static_assert(sizeof(mc::proto::ImuStatusPayload) == 12,
			  "ImuStatusPayload size (wire) must be 12 bytes");
static_assert(sizeof(mc::proto::Tsd20StatusPayload) == 8,
			  "Tsd20StatusPayload size (wire) must be 8 bytes");

static void sendStatus_(uint32_t now_ms) {
	const bool auto_active = (g_state.mode == mc::Mode::AUTO);
	const bool hb_timeout = auto_active && (g_state.last_hb_ms != 0) &&
							((uint32_t)(now_ms - g_state.last_hb_ms) >
							 (uint32_t)cfg::HEARTBEAT_TIMEOUT_MS);
	const bool have_cmd =
		(g_state.last_cmd_ms != 0) && (g_state.target_ttl_ms != 0);
	const uint32_t age_since_cmd_ms =
		have_cmd ? (uint32_t)(now_ms - g_state.last_cmd_ms) : 0;
	const bool ttl_expired =
		auto_active && have_cmd &&
		(age_since_cmd_ms > (uint32_t)g_state.target_ttl_ms);
	const bool auto_inactive =
		(!auto_active) && have_cmd &&
		(age_since_cmd_ms <= (uint32_t)g_state.target_ttl_ms);

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

	mc::proto::StatusPayload p{};
	p.seq_applied = (uint8_t)(g_state.last_applied_seq & 0xFFu);
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

	mc::proto::ImuStatusPayload p{};
	const int a_long =
		mc::clamp< int >((int)lroundf(st.a_long_mm_s2), -32768, 32767);
	const int v_est =
		mc::clamp< int >((int)lroundf(st.v_est_mm_s), -32768, 32767);
	wr16((uint8_t *)&p.a_long_mm_s2_le, (uint16_t)(int16_t)a_long);
	wr16((uint8_t *)&p.v_est_mm_s_le, (uint16_t)(int16_t)v_est);
	wr16((uint8_t *)&p.a_brake_cap_mm_s2_le,
		 (uint16_t)mc::clamp< int >((int)lroundf(st.a_brake_cap_mm_s2), 0,
									0xFFFF));
	const int yaw_x10 = (int)lroundf(st.gz_dps * 10.0f);
	wr16((uint8_t *)&p.yaw_rate_dps_x10_le,
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

static void sendTsd20Status_(uint32_t now_ms) {
	if (!cfg::TSD20_ENABLE)
		return;

	uint16_t age_ms = 0xFFFFu;
	if (g_tsd_last_read_ms != 0) {
		uint32_t age = now_ms - g_tsd_last_read_ms;
		if (age > 0xFFFFu)
			age = 0xFFFFu;
		age_ms = (uint16_t)age;
	}

	mc::proto::Tsd20StatusPayload p{};
	wr16((uint8_t *)&p.mm_le, g_tsd_state.mm);
	wr16((uint8_t *)&p.period_ms_le, g_tsd_period_ms);
	wr16((uint8_t *)&p.age_ms_le, age_ms);
	p.fail_count = g_tsd_state.fail_count;
	uint8_t flags = 0;
	if (g_tsd_state.ready)
		flags |= 1u << 0;
	if (g_tsd_state.valid)
		flags |= 1u << 1;
	p.flags = flags;

	uint8_t out[mc::proto::MAX_FRAME_ENCODED];
	size_t out_len = 0;
	mc::proto::PacketWriter::build(out, sizeof(out), out_len,
								   mc::proto::Type::TSD20_STATUS, 0, tsd_seq++,
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

static void updateTsd20_(uint32_t now_ms) {
	if (!cfg::TSD20_ENABLE)
		return;

	if (!g_tsd_state.ready && tsdInitTimer.due(now_ms)) {
		g_tsd_state.ready = tsd20.begin(tsd_uart);
		if (g_tsd_state.ready) {
			(void)tsd20.setLaser(true);
			g_tsd_state.fail_count = 0;
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

	if (!g_tsd_state.ready) {
		g_tsd_state.valid = false;
		g_tsd_state.mm = 0;
		g_tsd_last_read_ms = 0;
		g_tsd_period_ms = 0;
		return;
	}

	uint16_t mm = 0;
	if (tsd20.readDistanceMm(mm)) {
		g_tsd_state.mm = mm;
		g_tsd_state.valid = true;
		g_tsd_state.fail_count = 0;
		g_tsd_fail_logged = false;
		if (g_tsd_last_read_ms != 0) {
			const uint32_t dt = now_ms - g_tsd_last_read_ms;
			g_tsd_period_ms = (uint16_t)mc::clamp< uint32_t >(dt, 0u, 0xFFFFu);
		}
		g_tsd_last_read_ms = now_ms;
	} else {
		if (g_tsd_state.fail_count < 0xFF)
			g_tsd_state.fail_count++;
		if (g_tsd_state.fail_count >= cfg::TSD20_MAX_FAILS) {
			g_tsd_state.valid = false;
			g_tsd_state.mm = 0;
			if (!g_tsd_fail_logged) {
				alog.logf(mc::LogLevel::WARN, "tsd20", "read failed (fails=%u)",
						  (unsigned)g_tsd_state.fail_count);
				g_tsd_fail_logged = true;
			}
		}
	}
}

static void applyTargets_(uint32_t now_ms, float dt_s) {
	const bool abs_allowed =
		!g_state.killed &&
		(cfg::ABS_ENABLE &&
		 (g_state.mode == mc::Mode::AUTO || cfg::ABS_ENABLE_IN_MANUAL));
	const ImuEstimate &imu_state = imu_est.state();
	const bool imu_calib = g_imu_valid && imu_state.calibrated;
	const float v_est = g_imu_valid ? imu_state.v_est_mm_s : 0.0f;
	const bool speed_active = !g_state.killed;

	const AutoCommandResult desired = cmd_source.update(now_ms, g_state);
	if (desired.fresh) {
		// AUTOモードかつHB/TTLが有効で、今回のループで DRIVE
		// コマンドを実際に適用した。 このときのみ「適用された seq」として
		// last_applied_seq を更新する。
		g_state.last_applied_seq = g_state.last_seq;
	}
	const SafetyResult safe =
		safety.apply(now_ms, dt_s, desired.targets, g_state.mode, g_tsd_state,
					 imu_state, g_imu_valid, abs_allowed, &g_safety_diag);
	g_abs_active = safe.brake_mode;
	g_last_cmd_speed_mm_s = safe.targets.speed_mm_s;

	const SpeedControlOutput out = speed_ctl.update(
		safe.targets.speed_mm_s, v_est, dt_s, imu_calib, speed_active);
	g_speed_diag_v_cmd = (float)safe.targets.speed_mm_s;
	g_speed_diag_v_est = v_est;
	g_speed_diag_err = out.error_mm_s;
	g_speed_diag_i = out.integrator;
	g_speed_diag_pwm_cmd = out.pwm_cmd;
	g_speed_diag_pwm_ff = out.pwm_ff;
	g_speed_diag_saturated = out.saturated;
	g_speed_diag_active = out.active;
	g_speed_diag_calib = out.calibrated;

	drive.setBrakeMode(g_abs_active);
	drive.setTargetMmS(safe.targets.speed_mm_s, now_ms);
	drive.setTargetPwm(out.pwm_cmd, now_ms);
	drive.setTargetSteerCdeg(safe.targets.steer_cdeg, now_ms);
	drive.setTtlMs(safe.targets.ttl_ms, now_ms);
	drive.setDistMm(safe.targets.dist_mm, now_ms);

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

	drive.begin();

	uart_tx.begin(Serial2);
	alog.begin(uart_tx);
	alog.setMinLevel(mc::LogLevel::INFO);
	alog.logf(mc::LogLevel::INFO, "boot", "ESP32 up baud=%d",
			  (int)cfg::SERIAL_BAUD);

	g_tsd_state.ready = tsd20.begin(tsd_uart);
	if (g_tsd_state.ready) {
		(void)tsd20.setLaser(true);
		g_tsd_state.fail_count = 0;
		g_tsd_fail_logged = false;
		alog.logf(mc::LogLevel::INFO, "tsd20",
				  "init ok id=0x%02X swapped=%d freq_ack=%d iic_ack=%d",
				  (unsigned)tsd20.id(), (int)tsd20.swapped(),
				  (int)tsd20.freqAck(), (int)tsd20.iicAck());
	} else {
		alog.logf(mc::LogLevel::WARN, "tsd20", "init failed");
	}

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
	g_last_dt_s = dt_s;

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
		sendTsd20Status_(now_ms);
	}

	if (logTimer.due(now_ms)) {
		const char *mode =
			(g_state.mode == mc::Mode::MANUAL) ? "MANUAL" : "AUTO";
		alog.logf(mc::LogLevel::INFO, "drive",
				  "mode=%s killed=%d applied(speed=%dmm/s steer=%dcdeg "
				  "ttl=%ums dist=%umm) log_drop=%u uart_drop=%u",
				  mode, (int)g_state.killed, (int)drive.appliedSpeedMmS(),
				  (int)drive.appliedSteerCdeg(), (unsigned)drive.ttlMs(),
				  (unsigned)drive.distMm(), (unsigned)alog.dropped(),
				  (unsigned)uart_tx.dropped());
		if (cfg::TSD20_ENABLE) {
			alog.logf(mc::LogLevel::INFO, "tsd20",
					  "ready=%d valid=%d mm=%u fails=%u period=%ums "
					  "ack(freq=%d iic=%d)",
					  (int)g_tsd_state.ready, (int)g_tsd_state.valid,
					  (unsigned)g_tsd_state.mm,
					  (unsigned)g_tsd_state.fail_count,
					  (unsigned)g_tsd_period_ms, (int)tsd20.freqAck(),
					  (int)tsd20.iicAck());
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
		alog.logf(mc::LogLevel::INFO, "speed_ctl",
				  "active=%d calib=%d v_cmd=%.1f v_est=%.1f err=%.1f "
				  "pwm_cmd=%d pwm_ff=%d i=%.1f sat=%d",
				  (int)g_speed_diag_active, (int)g_speed_diag_calib,
				  (double)g_speed_diag_v_cmd, (double)g_speed_diag_v_est,
				  (double)g_speed_diag_err, (int)g_speed_diag_pwm_cmd,
				  (int)g_speed_diag_pwm_ff, (double)g_speed_diag_i,
				  (int)g_speed_diag_saturated);
		if (cfg::TSD20_ENABLE) {
			// tsd20 cap diagnostics, split into multiple short lines
			// to avoid truncation while preserving all fields.
			alog.logf(mc::LogLevel::INFO, "tsd20_cap",
					  "reason=%u clamp=%d margin=%.0f pred=%.0f steer=%.2f",
					  (unsigned)g_safety_diag.tsd.reason,
					  (int)g_safety_diag.tsd.clamped,
					  (double)g_safety_diag.tsd.margin_eff,
					  (double)g_safety_diag.tsd.margin_pred,
					  (double)g_safety_diag.tsd.steer_ratio);
			alog.logf(mc::LogLevel::INFO, "tsd20_cap",
					  "d_allow=%.0f d_travel=%.0f v_used=%.0f a_long=%.0f",
					  (double)g_safety_diag.tsd.d_allow,
					  (double)g_safety_diag.tsd.d_travel,
					  (double)g_safety_diag.tsd.v_est,
					  (double)g_safety_diag.tsd.a_long);
			alog.logf(mc::LogLevel::INFO, "tsd20_cap",
					  "a_cap=%.0f tau=%.3f v_max=%.1f v_cap=%.1f",
					  (double)g_safety_diag.tsd.a_cap,
					  (double)g_safety_diag.tsd.tau,
					  (double)g_safety_diag.tsd.v_max,
					  (double)g_safety_diag.tsd.v_cap);
		}
		if (cfg::ABS_ENABLE) {
			alog.logf(mc::LogLevel::INFO, "abs",
					  "reason=%u active=%d duty=%.2f v_cmd=%.1f v_est=%.1f "
					  "a_tgt=%.0f a_cap=%.0f decel=%.0f dt=%.3f",
					  (unsigned)g_safety_diag.abs.reason, (int)g_abs_active,
					  (double)g_safety_diag.abs.duty,
					  (double)g_safety_diag.abs.v_cmd,
					  (double)g_safety_diag.abs.v_est,
					  (double)g_safety_diag.abs.a_target,
					  (double)g_safety_diag.abs.a_cap,
					  (double)g_safety_diag.abs.decel, (double)g_last_dt_s);
		}
	}

	delay(1);
}
