// ABS 動作確認用 playground スケッチ
//
// 目的:
// - 壁に向かってまっすぐ進ませて、「減速・停止時の挙動」を観察する
// - 異なる ABS 風味のブレーキロジックを差し替えて比較する
//
// 前提:
// - 本番機と同じ配線（IBT-2 モータドライバ + サーボ + TSD20）を想定
// - ピン定義や PWM 設定は firmware 側の config/Config.h を再利用する
//
// 使い方（想定）:
// - シリアルモニタを 115200bps で接続
// - `m` キーで ABS モードをローテーション
// - `0`/`1`/`2` キーで ABS モードを直接選択
// - `g` キーで 1 本テスト走行を開始
//   - 前進開始 → 一定距離以下で ABS 開始 → 停止 or タイムアウトで終了
// - ログを見ながら「どのモードが一番安定して短距離で止まるか」を比較

#include <Arduino.h>
#include <Wire.h>

#include "config/Config.h"

// =======================
// TSD20 関連
// =======================

static TwoWire TsdWire(0);

static const uint8_t TSD20_ADDR = cfg::TSD20_I2C_ADDR;

static const uint8_t REG_TSD20_DIST_H = 0x00;
static const uint8_t REG_TSD20_LASER_CTRL = 0x02;
static const uint8_t REG_TSD20_ID = 0x03;

static const uint32_t TSD20_READ_INTERVAL_MS =
	cfg::TSD20_READ_INTERVAL_MS; // 5ms (=200Hz)

static bool i2cReadReg(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
	// repeated-start first
	TsdWire.beginTransmission(addr);
	TsdWire.write(reg);
	uint8_t rc = TsdWire.endTransmission(false);
	if (rc == 0) {
		size_t got = TsdWire.requestFrom((int)addr, (int)len, (int)true);
		if (got == len) {
			for (size_t i = 0; i < len; i++)
				buf[i] = (uint8_t)TsdWire.read();
			return true;
		}
	}

	// fallback STOP
	TsdWire.beginTransmission(addr);
	TsdWire.write(reg);
	rc = TsdWire.endTransmission(true);
	if (rc != 0)
		return false;

	size_t got = TsdWire.requestFrom((int)addr, (int)len, (int)true);
	if (got != len)
		return false;

	for (size_t i = 0; i < len; i++)
		buf[i] = (uint8_t)TsdWire.read();
	return true;
}

static bool i2cWrite8(uint8_t addr, uint8_t reg, uint8_t v) {
	TsdWire.beginTransmission(addr);
	TsdWire.write(reg);
	TsdWire.write(v);
	return (TsdWire.endTransmission(true) == 0);
}

static bool tsd20ReadDistanceMm(uint16_t &mm) {
	uint8_t b[2];
	if (!i2cReadReg(TSD20_ADDR, REG_TSD20_DIST_H, b, 2))
		return false;
	mm = (uint16_t(b[0]) << 8) | b[1];
	return true;
}

static bool tsd20ReadId(uint8_t &id) {
	return i2cReadReg(TSD20_ADDR, REG_TSD20_ID, &id, 1);
}

static bool tsd20Init() {
	TsdWire.end();
	delay(10);
	TsdWire.begin(cfg::TSD20_SDA_PIN, cfg::TSD20_SCL_PIN, cfg::TSD20_I2C_HZ);
	delay(30);

	uint8_t id = 0;
	if (!tsd20ReadId(id)) {
		Serial.println("[TSD20] I2C probe failed. 配線 / 電源 / UART "
					   "モードを確認してください。");
		return false;
	}

	Serial.printf("[TSD20] detected. ID=0x%02X\n", id);

	// レーザー ON を試行（失敗しても距離読み取りは試す）
	if (i2cWrite8(TSD20_ADDR, REG_TSD20_LASER_CTRL, 0x01)) {
		Serial.println("[TSD20] laser ON");
	} else {
		Serial.println(
			"[TSD20] laser ON failed (read only test will continue)");
	}
	return true;
}

// =======================
// モータ / ステアリング
// =======================

static void engineInit() {
	// IBT-2 enable pins
	pinMode(cfg::ENGINE_PIN_REN, OUTPUT);
	pinMode(cfg::ENGINE_PIN_LEN, OUTPUT);
	digitalWrite(cfg::ENGINE_PIN_REN, HIGH);
	digitalWrite(cfg::ENGINE_PIN_LEN, HIGH);

	// PWM channels
	ledcSetup(cfg::ENGINE_CHANNEL_RPWM, cfg::ENGINE_PWM_FREQ_HZ,
			  cfg::ENGINE_PWM_RES_BITS);
	ledcSetup(cfg::ENGINE_CHANNEL_LPWM, cfg::ENGINE_PWM_FREQ_HZ,
			  cfg::ENGINE_PWM_RES_BITS);
	ledcAttachPin(cfg::ENGINE_PIN_RPWM, cfg::ENGINE_CHANNEL_RPWM);
	ledcAttachPin(cfg::ENGINE_PIN_LPWM, cfg::ENGINE_CHANNEL_LPWM);

	ledcWrite(cfg::ENGINE_CHANNEL_RPWM, 0);
	ledcWrite(cfg::ENGINE_CHANNEL_LPWM, 0);
}

// [-255, 255] で指示。正: 前進、負: 逆転。
static void engineSetPwm(int pwm) {
	if (pwm > 255)
		pwm = 255;
	if (pwm < -255)
		pwm = -255;

	if (pwm > 0) {
		ledcWrite(cfg::ENGINE_CHANNEL_RPWM, pwm);
		ledcWrite(cfg::ENGINE_CHANNEL_LPWM, 0);
	} else if (pwm < 0) {
		ledcWrite(cfg::ENGINE_CHANNEL_RPWM, 0);
		ledcWrite(cfg::ENGINE_CHANNEL_LPWM, -pwm);
	} else {
		ledcWrite(cfg::ENGINE_CHANNEL_RPWM, 0);
		ledcWrite(cfg::ENGINE_CHANNEL_LPWM, 0);
	}
}

static void steerInitCenter() {
	// サーボをセンター (真っ直ぐ) に固定するだけ。
	ledcSetup(cfg::STEER_CHANNEL, cfg::STEER_PWM_FREQ_HZ,
			  cfg::STEER_PWM_RESOLUTION_BITS);
	ledcAttachPin(cfg::STEER_PIN_SERVO, cfg::STEER_CHANNEL);

	const uint32_t maxDuty = (1u << cfg::STEER_PWM_RESOLUTION_BITS) - 1u;
	const uint32_t duty = (uint32_t)((uint64_t)cfg::STEER_PULSE_CENTER_US *
									 maxDuty / cfg::STEER_PWM_PERIOD_US);
	ledcWrite(cfg::STEER_CHANNEL, duty);
}

// =======================
// ABS ロジック
// =======================

enum class AbsMode : uint8_t {
	NONE = 0,			// ABS なし（通常の「速度 0 だけ」ブレーキ）
	FULL_REVERSE = 1,	// 一定の逆転 PWM を入れっぱなし
	PULSED_REVERSE = 2, // 逆転 PWM を ON/OFF で断続的に入れる
};

static AbsMode g_mode = AbsMode::NONE;

// パラメータ（必要に応じて調整）
static const int PWM_FORWARD = 140;				 // 前進時の PWM
static const int PWM_REVERSE = 140;				 // 逆転ブレーキ用 PWM
static const uint16_t TRIGGER_DISTANCE_MM = 400; // この距離以下で ABS 開始
static const uint16_t STOP_DISTANCE_MM = 150;	 // これ以下で停止完了とみなす
static const uint32_t MAX_RUN_TIME_MS = 5000;	 // 1 本のテストの上限時間
static const uint32_t MAX_BRAKE_TIME_MS = 3000;	 // ABS フェーズの上限時間

// パルス ABS 用
static const uint32_t PULSE_PERIOD_MS = 120; // 1 サイクル長
static const uint32_t PULSE_ON_MS = 60;		 // うち ON している時間

// =======================
// 状態管理
// =======================

enum class RunState : uint8_t {
	IDLE = 0,
	FORWARD,
	BRAKE,
	DONE,
};

static RunState g_state = RunState::IDLE;

static bool g_tsdReady = false;

static uint32_t g_runStartMs = 0;
static uint32_t g_brakeStartMs = 0;
static uint16_t g_triggerDistanceMm = 0;

static uint32_t g_lastTsdPollMs = 0;

// パルス ABS 用
static bool g_pulseOn = false;
static uint32_t g_pulseLastToggleMs = 0;

// =======================
// Helper: 状態/モード表示
// =======================

static const char *modeName(AbsMode m) {
	switch (m) {
	case AbsMode::NONE:
		return "NONE(setSpeed0 only)";
	case AbsMode::FULL_REVERSE:
		return "FULL_REVERSE";
	case AbsMode::PULSED_REVERSE:
		return "PULSED_REVERSE";
	default:
		return "?";
	}
}

static const char *stateName(RunState s) {
	switch (s) {
	case RunState::IDLE:
		return "IDLE";
	case RunState::FORWARD:
		return "FORWARD";
	case RunState::BRAKE:
		return "BRAKE";
	case RunState::DONE:
		return "DONE";
	default:
		return "?";
	}
}

static void printBanner() {
	Serial.println();
	Serial.println("=== ABS Brake Playground ===");
	Serial.println("Controls:");
	Serial.println("  m : cycle ABS mode");
	Serial.println("  0 : mode = NONE (set speed 0 only)");
	Serial.println("  1 : mode = FULL_REVERSE");
	Serial.println("  2 : mode = PULSED_REVERSE");
	Serial.println("  g : start 1 run");
	Serial.println();
	Serial.printf("Initial ABS mode: %s\n", modeName(g_mode));
	Serial.println();
	Serial.println(
		"# log format: t_ms,state,dist_mm,mode,pwm,phase_ms,run_ms,extra");
}

static void setModeFromKey(char c) {
	AbsMode prev = g_mode;
	if (c == '0') {
		g_mode = AbsMode::NONE;
	} else if (c == '1') {
		g_mode = AbsMode::FULL_REVERSE;
	} else if (c == '2') {
		g_mode = AbsMode::PULSED_REVERSE;
	} else if (c == 'm') {
		// ローテーション
		if (g_mode == AbsMode::NONE)
			g_mode = AbsMode::FULL_REVERSE;
		else if (g_mode == AbsMode::FULL_REVERSE)
			g_mode = AbsMode::PULSED_REVERSE;
		else
			g_mode = AbsMode::NONE;
	} else {
		return;
	}

	if (g_mode != prev) {
		Serial.printf("[MODE] %s -> %s\n", modeName(prev), modeName(g_mode));
	}
}

static void tryStartRun() {
	if (!g_tsdReady) {
		Serial.println("[RUN] cannot start: TSD20 not ready");
		return;
	}
	if (g_state == RunState::FORWARD || g_state == RunState::BRAKE) {
		Serial.println("[RUN] already running");
		return;
	}

	g_state = RunState::FORWARD;
	g_runStartMs = millis();
	g_brakeStartMs = 0;
	g_triggerDistanceMm = 0;

	g_pulseOn = false;
	g_pulseLastToggleMs = 0;

	engineSetPwm(PWM_FORWARD);

	Serial.printf("[RUN] start: mode=%s, PWM_FORWARD=%d, PWM_REVERSE=%d\n",
				  modeName(g_mode), PWM_FORWARD, PWM_REVERSE);
}

// ABS 開始時の初期化
static void enterBrake(uint32_t nowMs, uint16_t distMm) {
	g_state = RunState::BRAKE;
	g_brakeStartMs = nowMs;
	g_triggerDistanceMm = distMm;

	g_pulseOn = false;
	g_pulseLastToggleMs = nowMs;

	Serial.printf("[ABS] enter: mode=%s, trigger_dist_mm=%u\n",
				  modeName(g_mode), (unsigned)distMm);

	// すぐに 1 回ロジックを適用しておく
	switch (g_mode) {
	case AbsMode::NONE:
		// 単純に「速度 0 にするだけ」
		engineSetPwm(0);
		break;
	case AbsMode::FULL_REVERSE:
		engineSetPwm(-PWM_REVERSE);
		break;
	case AbsMode::PULSED_REVERSE:
		engineSetPwm(-PWM_REVERSE);
		g_pulseOn = true;
		break;
	}
}

// ABS フェーズ中の PWM 更新
static void updateBrakePwm(uint32_t nowMs) {
	const uint32_t phaseMs =
		(g_brakeStartMs != 0) ? (nowMs - g_brakeStartMs) : 0;

	switch (g_mode) {
	case AbsMode::NONE:
		// 既に 0 にしているので何もしない
		engineSetPwm(0);
		break;

	case AbsMode::FULL_REVERSE:
		engineSetPwm(-PWM_REVERSE);
		break;

	case AbsMode::PULSED_REVERSE: {
		// ON/OFF パルス
		uint32_t sinceToggle = nowMs - g_pulseLastToggleMs;
		if (!g_pulseOn) {
			// OFF フェーズ
			if (sinceToggle >= (PULSE_PERIOD_MS - PULSE_ON_MS)) {
				g_pulseOn = true;
				g_pulseLastToggleMs = nowMs;
				engineSetPwm(-PWM_REVERSE);
			} else {
				engineSetPwm(0);
			}
		} else {
			// ON フェーズ
			if (sinceToggle >= PULSE_ON_MS) {
				g_pulseOn = false;
				g_pulseLastToggleMs = nowMs;
				engineSetPwm(0);
			} else {
				engineSetPwm(-PWM_REVERSE);
			}
		}
		break;
	}
	}

	// デバッグ用: まれにフェーズ時間を使いたい場合があるので保持だけしておく
	(void)phaseMs;
}

// =======================
// Arduino エントリ
// =======================

void setup() {
	Serial.begin(cfg::LOG_BAUD);
	delay(cfg::LOG_STARTUP_DELAY_MS);

	printBanner();

	// ハード初期化
	engineInit();
	steerInitCenter();

	g_tsdReady = tsd20Init();
	if (!g_tsdReady) {
		Serial.println(
			"[WARN] TSD20 not ready. "
			"距離センサなしでのテストは危険なので注意してください。");
	}

	g_state = RunState::IDLE;
	engineSetPwm(0);
}

void loop() {
	const uint32_t now = millis();

	// シリアル入力処理（ノンブロッキング）
	while (Serial.available() > 0) {
		char c = (char)Serial.read();
		if (c == 'g') {
			tryStartRun();
		} else {
			setModeFromKey(c);
		}
	}

	// 1 本のテストが長すぎる場合は強制停止
	if ((g_state == RunState::FORWARD || g_state == RunState::BRAKE) &&
		(now - g_runStartMs) > MAX_RUN_TIME_MS) {
		engineSetPwm(0);
		Serial.println("[RUN] timeout -> STOP");
		g_state = RunState::DONE;
	}

	// TSD20 ポーリング
	static uint16_t lastDistMm = 0;
	if ((now - g_lastTsdPollMs) >= TSD20_READ_INTERVAL_MS) {
		g_lastTsdPollMs = now;

		uint16_t mm = 0;
		bool ok = false;
		if (g_tsdReady) {
			ok = tsd20ReadDistanceMm(mm);
		}

		if (!ok) {
			// 読み取り失敗時は距離不明扱いでログだけ出す（安全のためモータは止める）
			engineSetPwm(0);
			if (g_state != RunState::IDLE) {
				Serial.printf(
					"LOG,%lu,%s,NA,%s,%d,%lu,%lu,READ_FAIL\n",
					(unsigned long)now, stateName(g_state), modeName(g_mode), 0,
					(unsigned long)(g_brakeStartMs ? (now - g_brakeStartMs)
												   : 0),
					(unsigned long)(g_runStartMs ? (now - g_runStartMs) : 0));
			}
			g_state = RunState::DONE;
			lastDistMm = 0;
			return;
		}

		lastDistMm = mm;

		// 状態遷移 & ABS ロジック
		if (g_state == RunState::FORWARD && mm <= TRIGGER_DISTANCE_MM) {
			enterBrake(now, mm);
		}

		if (g_state == RunState::BRAKE) {
			updateBrakePwm(now);

			const uint32_t phaseMs =
				(g_brakeStartMs != 0) ? (now - g_brakeStartMs) : 0;

			// 一定距離まで近づいた or 一定時間経過で停止完了扱い
			if (mm <= STOP_DISTANCE_MM || phaseMs > MAX_BRAKE_TIME_MS) {
				engineSetPwm(0);
				Serial.printf(
					"[ABS] done: mode=%s, trigger_mm=%u, stop_mm=%u, "
					"phase_ms=%lu, run_ms=%lu\n",
					modeName(g_mode), (unsigned)g_triggerDistanceMm,
					(unsigned)mm, (unsigned long)phaseMs,
					(unsigned long)(g_runStartMs ? (now - g_runStartMs) : 0));
				g_state = RunState::DONE;
			}
		}

		// ログ出力（統一フォーマット）
		int currentPwm = 0;
		// 現在の PWM を正確に読む API はないので、簡易的に:
		// - FORWARD 中は PWM_FORWARD
		// - BRAKE 中はモードごとに代表値を出す
		if (g_state == RunState::FORWARD) {
			currentPwm = PWM_FORWARD;
		} else if (g_state == RunState::BRAKE) {
			switch (g_mode) {
			case AbsMode::NONE:
				currentPwm = 0;
				break;
			case AbsMode::FULL_REVERSE:
				currentPwm = -PWM_REVERSE;
				break;
			case AbsMode::PULSED_REVERSE:
				currentPwm = g_pulseOn ? -PWM_REVERSE : 0;
				break;
			}
		} else {
			currentPwm = 0;
		}

		const uint32_t phaseMs =
			(g_brakeStartMs != 0) ? (now - g_brakeStartMs) : 0;
		const uint32_t runMs = (g_runStartMs != 0) ? (now - g_runStartMs) : 0;

		Serial.printf("LOG,%lu,%s,%u,%s,%d,%lu,%lu,\n", (unsigned long)now,
					  stateName(g_state), (unsigned)mm, modeName(g_mode),
					  currentPwm, (unsigned long)phaseMs, (unsigned long)runMs);
	}

	// ループをあまり詰めすぎない
	delay(1);
}
