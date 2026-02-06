// TSD20 の「読み取り側から見た実効更新レート」を観察するための
// playground 用スケッチ。
//
// 制約 / 注意:
// - このコードは「ESP32 が I2C で読みに行った周期」と
//   「読み取った距離値が変化したタイミング」しか観測できません。
// - TSD20 内部の実際の更新周期がもっと遅くても、
//   たまたま同じ値を繰り返し返しているだけの場合、
//   コード上の「read OK 周期」は 5ms (200Hz) に見える可能性があります。
// - そのため、「値が変化したタイミングの間隔」から
//   おおよその更新レートを推定する、という形になります。
//
// 使い方の例:
// - センサ前に距離が変化するターゲット（振り子や前後に動かす板など）を置き、
//   ログを PC 側でキャプチャして、値変化の間隔と Hz を確認してください。

#include <Arduino.h>
#include <Wire.h>

// =======================
// Pin / Config
// =======================

// 本番ファームと合わせた TSD20 I2C ピン
static const int PIN_TSD20_SDA = 32;
static const int PIN_TSD20_SCL = 33;

// I2C
static const uint32_t I2C_FREQ_HZ = 400000;
static const uint8_t TSD20_ADDR = 0x52;

// レジスタ
static const uint8_t REG_DIST_H = 0x00;
static const uint8_t REG_LASER_CTRL = 0x02;
static const uint8_t REG_ID = 0x03;

// 読み取り間隔（ESP32 側ポーリング周期）
// 本番ファームの cfg::TSD20_READ_INTERVAL_MS と同じ 5ms (=200Hz) に設定。
static const uint32_t READ_INTERVAL_MS = 5;

// =======================
// I2C helpers
// =======================

static bool i2cReadReg(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
	// repeated-start first
	Wire.beginTransmission(addr);
	Wire.write(reg);
	uint8_t rc = Wire.endTransmission(false);
	if (rc == 0) {
		size_t got = Wire.requestFrom((int)addr, (int)len, (int)true);
		if (got == len) {
			for (size_t i = 0; i < len; i++)
				buf[i] = (uint8_t)Wire.read();
			return true;
		}
	}

	// fallback STOP
	Wire.beginTransmission(addr);
	Wire.write(reg);
	rc = Wire.endTransmission(true);
	if (rc != 0)
		return false;

	size_t got = Wire.requestFrom((int)addr, (int)len, (int)true);
	if (got != len)
		return false;

	for (size_t i = 0; i < len; i++)
		buf[i] = (uint8_t)Wire.read();
	return true;
}

static bool i2cWrite8(uint8_t addr, uint8_t reg, uint8_t v) {
	Wire.beginTransmission(addr);
	Wire.write(reg);
	Wire.write(v);
	return (Wire.endTransmission(true) == 0);
}

static bool tsd20ReadDistanceMm(uint16_t &mm) {
	uint8_t b[2];
	if (!i2cReadReg(TSD20_ADDR, REG_DIST_H, b, 2))
		return false;
	mm = (uint16_t(b[0]) << 8) | b[1];
	return true;
}

static bool tsd20ReadId(uint8_t &id) {
	return i2cReadReg(TSD20_ADDR, REG_ID, &id, 1);
}

// =======================
// State
// =======================

static bool g_tsd20Ready = false;

// 直近の読み取り結果
static uint16_t g_lastMm = 0;
static bool g_hasLastMm = false;

// 「値が変化した瞬間」の統計
static uint32_t g_lastChangeMs = 0;
static bool g_hasLastChange = false;
static uint32_t g_changeCount = 0;
static double g_changeIntervalSumMs = 0.0;

// 全読み取り試行の統計
static uint32_t g_readOkCount = 0;
static uint32_t g_readFailCount = 0;

// =======================
// Helpers
// =======================

static void initI2C() {
	Wire.end();
	delay(10);
	Wire.begin(PIN_TSD20_SDA, PIN_TSD20_SCL, I2C_FREQ_HZ);
	delay(30);
}

static void printBanner() {
	Serial.println("=== TSD20 Rate Probe (I2C, 5ms poll) ===");
	Serial.printf("Pins: SDA=GPIO%d, SCL=GPIO%d\n", PIN_TSD20_SDA,
				  PIN_TSD20_SCL);
	Serial.printf("I2C: addr=0x%02X freq=%lu Hz\n", TSD20_ADDR,
				  (unsigned long)I2C_FREQ_HZ);
	Serial.printf("Poll interval: %lu ms (target ~%lu Hz)\n",
				  (unsigned long)READ_INTERVAL_MS,
				  (unsigned long)(1000u / READ_INTERVAL_MS));
	Serial.println();
	Serial.println(
		"# format: "
		"READ,<ms>,<mm>,<is_new>,<dt_change_ms>,<est_change_hz>,<ok>,<"
		"fail>");
	Serial.println(
		"# 注意: is_new=0 が続いても、TSD20 内部更新が止まっているとは限りま"
		"せん（同じ値を返している可能性があります）。");
	Serial.println(
		"#       動くターゲットで実験し、「値が変わるタイミング」の間隔を見て更"
		"新レートを推定してください。");
	Serial.println();
}

// =======================
// Arduino entry points
// =======================

void setup() {
	Serial.begin(115200);
	delay(200);

	printBanner();

	// I2C 初期化 & ID 読み取り
	initI2C();
	uint8_t id = 0;
	bool id_ok = tsd20ReadId(id);
	if (id_ok) {
		Serial.printf("[TSD20] I2C detected. ID=0x%02X\n", id);
		g_tsd20Ready = true;
	} else {
		Serial.println(
			"[TSD20] I2C probe failed. 配線 / 電源 / UARTモードを確認してくださ"
			"い。");
		g_tsd20Ready = false;
	}

	// レーザー ON を試行（失敗しても読み取りは試す）
	if (g_tsd20Ready) {
		if (i2cWrite8(TSD20_ADDR, REG_LASER_CTRL, 0x01)) {
			Serial.println("[TSD20] laser ON");
		} else {
			Serial.println(
				"[TSD20] laser ON failed (read only test will continue)");
		}
	}

	Serial.println();
}

void loop() {
	static uint32_t lastPollMs = 0;
	const uint32_t now = millis();

	if (now - lastPollMs < READ_INTERVAL_MS) {
		return;
	}
	lastPollMs = now;

	if (!g_tsd20Ready) {
		// 検出に失敗している場合でも、無駄に I2C を叩き続けない。
		return;
	}

	uint16_t mm = 0;
	const bool ok = tsd20ReadDistanceMm(mm);
	if (ok) {
		g_readOkCount++;
	} else {
		g_readFailCount++;
		Serial.printf("READ_FAIL,%lu,%lu,%lu\n", (unsigned long)now,
					  (unsigned long)g_readOkCount,
					  (unsigned long)g_readFailCount);
		return;
	}

	bool is_new = false;
	double dt_change_ms = 0.0;
	double est_change_hz = 0.0;

	if (!g_hasLastMm || mm != g_lastMm) {
		is_new = true;
		if (g_hasLastChange) {
			const uint32_t dt = now - g_lastChangeMs;
			dt_change_ms = (double)dt;
			g_changeIntervalSumMs += (double)dt;
			g_changeCount++;
			const double avg_ms = g_changeIntervalSumMs / (double)g_changeCount;
			if (avg_ms > 0.0) {
				est_change_hz = 1000.0 / avg_ms;
			}
		} else {
			// 最初の変化なので統計はまだなし
			g_changeCount = 0;
			g_changeIntervalSumMs = 0.0;
		}
		g_lastChangeMs = now;
		g_hasLastChange = true;
	}

	g_lastMm = mm;
	g_hasLastMm = true;

	// ログ出力
	// READ,<ms>,<mm>,<is_new>,<dt_change_ms>,<est_change_hz>,<ok>,<fail>
	Serial.printf("READ,%lu,%u,%d,%.3f,%.3f,%lu,%lu\n", (unsigned long)now, mm,
				  is_new ? 1 : 0, dt_change_ms, est_change_hz,
				  (unsigned long)g_readOkCount, (unsigned long)g_readFailCount);
}
