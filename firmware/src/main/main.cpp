#include "../control/ControllerInput.h"
#include "../hardware/Drive.h"
#include "../log/Logger.h"
#include <Arduino.h>
#include <vector>

static ControllerInput pad;
static Drive drive;
static Logger logg;

static bool running = true;
static int speedNow = 0;

static const int SPEED_MAX = 180;
static const int SPEED_STEP = 6;
static const int LOOP_DELAY = 20;

static const int SERIAL_TXD = 16;
static const int SERIAL_RXD = 17;

struct input {
	int dist;
	int deg;
};

void setup() {
	logg.begin(115200);

	Serial2.begin(115200, SERIAL_8N1, SERIAL_RXD, SERIAL_TXD);

	Serial.println("\n=== ESP32 Bluepad32: One-Line Logger ===");

	pad.begin();
	drive.begin();
}

void target_update() {
	std::vector< input > inputs;
	// シリアルから最大360個のデータを受け取ります
	while (Serial2.available() > 0 && inputs.size() < 360) {
		String line = Serial2.readStringUntil('\n');
		Serial.println(line);
		if (line.length() == 0)
			continue;

		int commaPos = line.indexOf(',');
		if (commaPos < 0)
			continue;

		String sDist = line.substring(0, commaPos);
		String sDeg = line.substring(commaPos + 1);

		int dist = sDist.toInt();
		int deg = sDeg.toInt();

		inputs.push_back({dist, deg});
	}
	// データを評価
	for (const auto &x : inputs) {
		drive.evalInput(x.dist, x.deg);
	}
	// スピードとステアを適用
	drive.control();
	Serial.println(drive.info());
}

void loop() { target_update(); }

// void pad_update() {
// 	pad.update();

// 	bool connected = pad.isConnected();

// 	if (!connected) {
// 		speedNow = 0;
// 		drive.stop();
// 		steer.center();

// 		static PadState dummy{};
// 		logg.printLine(false, running, dummy, 0, "CENTER");

// 		delay(LOOP_DELAY);
// 		return;
// 	}

// 	// '-' で Pause / Resume
// 	if (pad.consumeToggleRunning()) {
// 		running = !running;
// 		if (!running) {
// 			speedNow = 0;
// 			drive.stop();
// 			steer.center();
// 		}
// 	}

// 	const PadState &st = pad.state();
// 	const char *steerStr = "CENTER";

// 	if (running) {
// 		// --- ステア ---
// 		if (st.dpad & 0x04) {
// 			steer.right();
// 			steerStr = "RIGHT";
// 		} else if (st.dpad & 0x08) {
// 			steer.left();
// 			steerStr = "LEFT";
// 		} else {
// 			steer.center();
// 		}

// 		// --- 走行 ---
// 		if (st.B) {
// 			speedNow -= SPEED_STEP;
// 		} else if (st.A) {
// 			speedNow += SPEED_STEP;
// 		} else {
// 			speedNow -= SPEED_STEP;
// 		}

// 		speedNow = constrain(speedNow, 0, SPEED_MAX);

// 		if (speedNow == 0)
// 			drive.stop();
// 		else
// 			drive.setSpeed(speedNow);
// 	}

// 	// ★ ここが一行ログ本体
// 	logg.printLine(true, running, st, speedNow, steerStr);

// 	delay(LOOP_DELAY);
// }
