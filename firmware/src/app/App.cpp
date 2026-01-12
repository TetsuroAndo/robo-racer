#include "app/App.h"
#include <common/Math.h>

namespace mc {

App::App()
	: _ble(), _commands(Serial, cfg::BLE_ENABLED ? &_ble : nullptr),
	  _actuators(cfg::PIN_RPWM, cfg::PIN_LPWM, cfg::PIN_REN, cfg::PIN_LEN,
				 cfg::PIN_SERVO),
	  _serialLogSink(Serial), _bleLogSink(_ble), _uartProtoLogSink(Serial),
	  _udpLogSink(), _hilsSerialSink(Serial), _hilsBleSink(_ble) {}

void App::begin() {
	Serial.begin(cfg::SERIAL_BAUD);
	delay(50);

	configureLogging();

	MC_LOGI(log::Topic::App, "rc firmware boot");
	MC_LOGI(log::Topic::App,
			"cmd: <throttle> <steer> (floats -1..1 or ints -1000..1000)");

	_actuators.begin(cfg::SERVO_MIN_US, cfg::SERVO_MAX_US,
					 cfg::SERVO_CENTER_US);

	if (cfg::BLE_ENABLED) {
		_ble.begin(cfg::BLE_DEVICE_NAME, cfg::BLE_MTU);
		MC_LOGI(log::Topic::Comm, "BLE UART ready");
	}

	_commands.begin();
	configureHils();

	const uint32_t now = millis();
	_actuators.apply(0.0f, 0.0f, now);
	_lastCmdMs = 0;
	_timeoutActive = false;
}

void App::configureLogging() {
	log::Logger &logger = log::Logger::instance();
	logger.clearSinks();
	logger.setLevelAll(log::levelFromInt(cfg::LOG_LEVEL_DEFAULT));
	logger.setLevel(log::Topic::Comm, log::levelFromInt(cfg::LOG_LEVEL_COMM));
	logger.setLevel(log::Topic::Control,
					log::levelFromInt(cfg::LOG_LEVEL_CONTROL));
	logger.setLevel(log::Topic::Hardware,
					log::levelFromInt(cfg::LOG_LEVEL_HARDWARE));
	logger.setLevel(log::Topic::Hils, log::levelFromInt(cfg::LOG_LEVEL_HILS));

	if (cfg::LOG_TO_SERIAL)
		logger.attachSink(&_serialLogSink);
	if (cfg::BLE_ENABLED && cfg::LOG_TO_BLE)
		logger.attachSink(&_bleLogSink);
	if (cfg::LOG_TO_UART_PROTOCOL)
		logger.attachSink(&_uartProtoLogSink);
	if (cfg::LOG_TO_UDP) {
		if (_udpLogSink.begin(cfg::WIFI_SSID, cfg::WIFI_PASS, cfg::LOG_UDP_HOST,
							  cfg::LOG_UDP_PORT,
							  cfg::LOG_UDP_CONNECT_TIMEOUT_MS)) {
			logger.attachSink(&_udpLogSink);
		} else {
			MC_LOGW(log::Topic::Comm, "udp log disabled");
		}
	}
}

static OutputMode outputModeFromConfig() {
	switch (cfg::OUTPUT_MODE) {
	case 1:
		return OutputMode::Shadow;
	case 2:
		return OutputMode::Hils;
	default:
		return OutputMode::Hardware;
	}
}

void App::configureHils() {
	const OutputMode mode = outputModeFromConfig();
	_actuators.setOutputMode(mode);

	if (mode == OutputMode::Hardware)
		return;

	bool attached = false;
	_hilsMux.clear();

	if (cfg::HILS_REPORT_TO_SERIAL) {
		_hilsMux.attach(&_hilsSerialSink);
		attached = true;
	}
	if (cfg::BLE_ENABLED && cfg::HILS_REPORT_TO_BLE) {
		_hilsMux.attach(&_hilsBleSink);
		attached = true;
	}

	_actuators.attachHilsSink(&_hilsMux);

	if (!attached)
		MC_LOGW(log::Topic::Hils, "HILS mode set but no sink configured");
	else
		MC_LOGI(log::Topic::Hils, "HILS output mode active");
}

bool App::readInputs(uint32_t nowMs) {
	const bool updated = _commands.poll(_cmd);
	if (updated) {
		_lastCmdMs = nowMs;
		if (_timeoutActive)
			MC_LOGI(log::Topic::Control, "host recovered");
		_timeoutActive = false;
	}
	return updated;
}

void App::applyOutputs(const comm::RcCommand &cmd, uint32_t nowMs) {
	const float input = mc::clamp(cmd.throttle, -1.0f, 1.0f);
	const float scale =
		(input >= 0.0f) ? cfg::THROTTLE_MAX : -cfg::THROTTLE_REVERSE;
	const float throttle = input * scale;
	const float steer = mc::clamp(cmd.steer, -1.0f, 1.0f) * cfg::STEER_MAX;

	_actuators.apply(throttle, steer, nowMs);
}

void App::loop() {
	const uint32_t now = millis();
	const bool updated = readInputs(now);

	if (updated) {
		MC_LOGD(log::Topic::Control, "cmd: %.3f %.3f", _cmd.throttle,
				_cmd.steer);
	}

	if (!updated && _lastCmdMs != 0 &&
		(now - _lastCmdMs) > cfg::HOST_HEARTBEAT_TIMEOUT_MS) {
		if (!_timeoutActive)
			MC_LOGW(log::Topic::Control, "host timeout, stopping");
		_timeoutActive = true;
		_cmd = comm::RcCommand{};
	}

	applyOutputs(_cmd, now);
}

} // namespace mc
