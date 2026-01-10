#include "app/App.h"
#include <common/Math.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>

namespace mc {

App::App() {}

void App::begin() {
	Serial.begin(cfg::SERIAL_BAUD);
	delay(50);
	Serial.println("rc firmware boot");
	Serial.println(
		"cmd: <throttle> <steer>  (floats -1..1 or ints -1000..1000)");

	_motor.begin();
	_servo.begin(cfg::SERVO_MIN_US, cfg::SERVO_MAX_US, cfg::SERVO_CENTER_US);

	if (cfg::BLE_ENABLED) {
		_ble.begin(cfg::BLE_DEVICE_NAME, cfg::BLE_MTU);
		Serial.println("BLE UART ready");
	}

	_motor.stop();
	_servo.center();
	_lastCmdMs = 0;
}

static float normalizeInput(float v) {
	if (fabsf(v) <= 1.2f)
		return v;
	return v / 1000.0f;
}

bool App::LineParser::parseLine(const char *line, RcCommand &out) {
	float values[2] = {0.0f, 0.0f};
	int found = 0;
	const char *p = line;

	while (*p && found < 2) {
		while (*p && !isdigit((unsigned char)*p) && *p != '+' && *p != '-' &&
			   *p != '.') {
			++p;
		}
		if (!*p)
			break;
		char *end = nullptr;
		const float v = strtof(p, &end);
		if (end == p)
			break;
		values[found++] = v;
		p = end;
	}

	if (found < 2)
		return false;

	out.throttle = mc::clamp(normalizeInput(values[0]), -1.0f, 1.0f);
	out.steer = mc::clamp(normalizeInput(values[1]), -1.0f, 1.0f);
	return true;
}

bool App::LineParser::poll(Stream &stream, RcCommand &out) {
	bool updated = false;
	while (stream.available() > 0) {
		const int c = stream.read();
		if (c < 0)
			break;

		if (c == '\n' || c == '\r') {
			if (_len > 0) {
				_buf[_len] = '\0';
				if (parseLine(_buf, out))
					updated = true;
				_len = 0;
			}
			continue;
		}

		if (_len + 1 >= kBufSize) {
			_len = 0;
			continue;
		}

		_buf[_len++] = (char)c;
	}

	return updated;
}

bool App::readInputs(uint32_t nowMs) {
	bool updated = false;

	if (_serialParser.poll(Serial, _cmd)) {
		_lastCmdMs = nowMs;
		updated = true;
	}

	if (cfg::BLE_ENABLED) {
		_ble.poll();
		if (_bleParser.poll(_ble, _cmd)) {
			_lastCmdMs = nowMs;
			updated = true;
		}
	}

	return updated;
}

void App::applyOutputs(const RcCommand &cmd) {
	const float input = mc::clamp(cmd.throttle, -1.0f, 1.0f);
	const float scale =
		(input >= 0.0f) ? cfg::THROTTLE_MAX : -cfg::THROTTLE_REVERSE;
	const float throttle = input * scale;
	const float steer = mc::clamp(cmd.steer, -1.0f, 1.0f) * cfg::STEER_MAX;

	_motor.setThrottle(throttle);
	_servo.setNormalized(steer);
}

void App::loop() {
	const uint32_t now = millis();
	const bool updated = readInputs(now);

	if (!updated && _lastCmdMs != 0 &&
		(now - _lastCmdMs) > cfg::HOST_HEARTBEAT_TIMEOUT_MS) {
		_cmd = RcCommand{};
	}

	applyOutputs(_cmd);
}

} // namespace mc
