// PWM 段階出力テスト
//
// 目的:
// - モータにさまざまな PWM を段階的に出して動作確認する
// - SPEED_PWM_MIN_FORWARD(55), DRIVE_PWM_MIN_WHEN_STOP(32) などの
//   閾値で発進するかどうかを実機で確認する
//
// 前提:
// - 本番機と同じ配線（IBT-2 モータドライバ）
// - ピン定義は firmware/src/config/Config.h を再利用
//
// 使い方:
// - platformio run -t upload で ESP32 に書き込み
// - シリアルモニタ 115200bps で接続
// - `s` キー: スイープ開始（0→順に増加→255→停止）
// - `r` キー: 逆スイープ（255→順に減少→0）
// - `0`〜`9` キー: 固定 PWM 値（0,30,55,70,100,128,150,200,255）
// - ` ` キー: 停止
// - `h` キー: ヘルプ表示

#include <Arduino.h>

#include "config/Config.h"

//------------------------------------------------------------------------------
// モータ制御
//------------------------------------------------------------------------------

static void engineInit() {
	pinMode(cfg::ENGINE_PIN_REN, OUTPUT);
	pinMode(cfg::ENGINE_PIN_LEN, OUTPUT);
	digitalWrite(cfg::ENGINE_PIN_REN, HIGH);
	digitalWrite(cfg::ENGINE_PIN_LEN, HIGH);

	ledcSetup(cfg::ENGINE_CHANNEL_RPWM, cfg::ENGINE_PWM_FREQ_HZ,
			  cfg::ENGINE_PWM_RES_BITS);
	ledcSetup(cfg::ENGINE_CHANNEL_LPWM, cfg::ENGINE_PWM_FREQ_HZ,
			  cfg::ENGINE_PWM_RES_BITS);
	ledcAttachPin(cfg::ENGINE_PIN_RPWM, cfg::ENGINE_CHANNEL_RPWM);
	ledcAttachPin(cfg::ENGINE_PIN_LPWM, cfg::ENGINE_CHANNEL_LPWM);

	ledcWrite(cfg::ENGINE_CHANNEL_RPWM, 0);
	ledcWrite(cfg::ENGINE_CHANNEL_LPWM, 0);
}

// 前進: pwm [0..255], 0 は停止
static void engineSetForward(uint8_t pwm) {
	ledcWrite(cfg::ENGINE_CHANNEL_RPWM, pwm);
	ledcWrite(cfg::ENGINE_CHANNEL_LPWM, 0);
}

static void engineStop() {
	ledcWrite(cfg::ENGINE_CHANNEL_RPWM, 0);
	ledcWrite(cfg::ENGINE_CHANNEL_LPWM, 0);
}

//------------------------------------------------------------------------------
// 閾値ポイント（Config の参考値）
//------------------------------------------------------------------------------

static const uint8_t PWM_STEPS[] = {0, 30, 55, 70, 100, 128, 150, 200, 255};
//  ^    ^    ^
//  |    |    SPEED_PWM_MIN_FORWARD
//  |    DRIVE_PWM_MIN_WHEN_STOP 付近
//  停止

static const uint8_t PWM_STEPS_SIZE = sizeof(PWM_STEPS) / sizeof(PWM_STEPS[0]);

//------------------------------------------------------------------------------
// スイープ状態
//------------------------------------------------------------------------------

enum class SweepMode : uint8_t {
	IDLE,
	FORWARD, // 0 → 255
	REVERSE, // 255 → 0
};

static SweepMode g_sweep_mode = SweepMode::IDLE;
static uint8_t g_current_pwm = 0;
static uint32_t g_last_step_ms = 0;
static const uint32_t STEP_DWELL_MS = 800; // 各 PWM で 800ms 維持

//------------------------------------------------------------------------------
// シリアルヘルプ
//------------------------------------------------------------------------------

static void printHelp() {
	Serial.println();
	Serial.println("=== PWM Sweep Test ===");
	Serial.println("  s: sweep 0..255 (forward)");
	Serial.println("  r: sweep 255..0 (reverse)");
	Serial.println("  1-9: fixed PWM (30,55,70,100,128,150,200,255)");
	Serial.println("  [space]: stop");
	Serial.println("  h: this help");
	Serial.printf("  Thresholds: SPEED_PWM_MIN=%d DRIVE_PWM_MIN=%d\n",
				  cfg::SPEED_PWM_MIN_FORWARD, cfg::DRIVE_PWM_MIN_WHEN_STOP);
	Serial.println();
}

//------------------------------------------------------------------------------
// setup / loop
//------------------------------------------------------------------------------

void setup() {
	Serial.begin(115200);
	delay(200);

	Serial.println();

	engineInit();

	Serial.println("PWM Sweep Test ready.");
	printHelp();
}

void loop() {
	const uint32_t now_ms = millis();

	// シリアル入力
	if (Serial.available() > 0) {
		int c = Serial.read();
		switch (c) {
		case 's':
			g_sweep_mode = SweepMode::FORWARD;
			g_current_pwm = 0;
			g_last_step_ms = now_ms;
			engineStop();
			Serial.println("Sweep FORWARD: 0 -> 255");
			break;
		case 'r':
			g_sweep_mode = SweepMode::REVERSE;
			g_current_pwm = 255;
			g_last_step_ms = now_ms;
			engineSetForward(255);
			Serial.println("Sweep REVERSE: 255 -> 0");
			break;
		case ' ':
		case '0':
			g_sweep_mode = SweepMode::IDLE;
			engineStop();
			Serial.println("Stop");
			break;
		case 'h':
		case '?':
			printHelp();
			break;
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
		case '6':
		case '7':
		case '8':
		case '9': {
			int idx = c - '1'; // 1->0, 2->1, ...
			if (idx >= 0 && idx < PWM_STEPS_SIZE) {
				g_sweep_mode = SweepMode::IDLE;
				g_current_pwm = PWM_STEPS[idx];
				engineSetForward(g_current_pwm);
				Serial.printf("Fixed PWM=%u\n", g_current_pwm);
			}
			break;
		}
		default:
			break;
		}
	}

	// スイープ進行
	if (g_sweep_mode != SweepMode::IDLE &&
		(now_ms - g_last_step_ms) >= STEP_DWELL_MS) {
		g_last_step_ms = now_ms;

		if (g_sweep_mode == SweepMode::FORWARD) {
			if (g_current_pwm >= 255) {
				g_sweep_mode = SweepMode::IDLE;
				engineStop();
				Serial.println("Sweep done. PWM=0.");
			} else {
				g_current_pwm++;
				engineSetForward(g_current_pwm);
				if (g_current_pwm % 20 == 0 || g_current_pwm <= 60) {
					Serial.printf("  PWM=%u\n", g_current_pwm);
				}
			}
		} else if (g_sweep_mode == SweepMode::REVERSE) {
			if (g_current_pwm == 0) {
				g_sweep_mode = SweepMode::IDLE;
				engineStop();
				Serial.println("Sweep done. PWM=0.");
			} else {
				g_current_pwm--;
				engineSetForward(g_current_pwm);
				if (g_current_pwm % 20 == 0 ||
					(g_current_pwm <= 60 && g_current_pwm > 0)) {
					Serial.printf("  PWM=%u\n", g_current_pwm);
				}
			}
		}
	}

	delay(10);
}
