#include "app/App.h"
#include <common/Math.h>

namespace mc {

App::App() {
	_rs.mode = proto::Mode::Auto;
	_rs.preferRight = cfg::PREFER_RIGHT_TURN_DEFAULT;
}

void App::begin() {
	Serial.begin(cfg::SERIAL_BAUD);
	delay(50);
	Serial.println("mc firmware boot");

	Wire.begin(cfg::PIN_I2C_SDA, cfg::PIN_I2C_SCL, cfg::I2C_FREQ_HZ);

	_motor.begin();
	_servo.begin(cfg::SERVO_MIN_US, cfg::SERVO_MAX_US, cfg::SERVO_CENTER_US);

	_commSerial.begin(Serial);
	if (cfg::BLE_ENABLED) {
		_ble.begin(cfg::BLE_DEVICE_NAME, cfg::BLE_MTU);
		_commBle.begin(_ble);
		Serial.println("BLE UART ready");
	}

	// Sensors
	auto r1 = _tsd.begin(Wire);
	if (!r1.ok())
		Serial.println("TSD20 not ready (I2C) - check wiring/address");

	_tsd.setLaserEnabled(true);

	auto r2 = _imu.begin(Wire);
	if (!r2.ok())
		Serial.println("MPU6050 not ready - check wiring/address");
	else
		Serial.println("MPU6050 OK (keep still 3-5s for bias)");

	_lastLoopMs = millis();
	_loopCounterTs = _lastLoopMs;
	_telemetryTimer.reset(_lastLoopMs);

	// Safe outputs
	_motor.stop();
	_servo.center();
	_motion.reset();
}

void App::updateSensors() {
	// TSD20
	uint16_t mm = 0;
	const auto rd = _tsd.readDistanceMm(mm);
	if (rd.ok()) {
		_sensors.distValid = true;
		_sensors.distMm = mm;
	} else {
		_sensors.distValid = false;
		_sensors.distMm = 0;
	}

	// IMU
	if (_imu.update()) {
		_sensors.imuValid = _imu.healthy();
		_sensors.yawDeg = _imu.yawDeg();
		_sensors.yawRateDegS = _imu.yawRateDegS();
	} else {
		_sensors.imuValid = _imu.healthy();
	}
}

void App::applyHostCommands(uint32_t nowMs) {
	_commSerial.poll();
	const HostCommand cSerial = _commSerial.consumeCommand();

	HostCommand cBle{};
	uint32_t lastBle = 0;
	if (cfg::BLE_ENABLED) {
		_ble.poll();
		_commBle.poll();
		cBle = _commBle.consumeCommand();
		lastBle = _commBle.lastHostSeenMs();
	}

	const uint32_t lastSerial = _commSerial.lastHostSeenMs();

	if (cSerial.estop || cBle.estop) {
		_rs.estop = EstopState::Active;
	}
	if (cSerial.clearEstop || cBle.clearEstop) {
		_rs.estop = EstopState::Clear;
		_motion.reset();
	}

	const bool serialAlive =
		(lastSerial != 0) &&
		((nowMs - lastSerial) <= cfg::HOST_HEARTBEAT_TIMEOUT_MS);
	const bool bleAlive =
		(lastBle != 0) && ((nowMs - lastBle) <= cfg::HOST_HEARTBEAT_TIMEOUT_MS);

	const HostCommand *active = nullptr;
	if (bleAlive && (!serialAlive || lastBle >= lastSerial)) {
		active = &cBle;
	} else if (serialAlive) {
		active = &cSerial;
	}

	if (active) {
		if (active->hasPrefer) {
			_rs.preferRight = active->preferRight;
		}
		if (active->hasMode) {
			_rs.mode = active->mode;
			_rs.hasHeadingRef = false;
			_motion.reset();
		}
		if (active->hasManual) {
			// Keep mode as Manual if the host is driving; but we don't force
			// it.
			_target.throttle = active->throttle;
			_target.steer = active->steer;
		}
	}

	uint32_t lastHost = lastSerial;
	if (lastBle > lastHost)
		lastHost = lastBle;

	// Host watchdog: if host selected Manual but stopped talking, stop.
	const bool hostTimedOut =
		(lastHost != 0) &&
		((nowMs - lastHost) > cfg::HOST_HEARTBEAT_TIMEOUT_MS);
	if (hostTimedOut && _rs.mode == proto::Mode::Manual) {
		_target = ControlCommand{};
	}
}

void App::runStateMachine(uint32_t nowMs, float dt_s) {
	// Hard safety: ESTOP always wins
	if (_rs.estop == EstopState::Active) {
		_target = ControlCommand{};
		_out = ControlCommand{};
		return;
	}

	// Decide target based on mode
	if (_rs.mode == proto::Mode::Idle) {
		_target = ControlCommand{};
	} else if (_rs.mode == proto::Mode::Manual) {
		// _target already updated by host, but constrain safety based on
		// distance
		if (_sensors.distValid &&
			_sensors.distMm <= cfg::DIST_EMERGENCY_STOP_MM) {
			_target.throttle = 0.0f;
		}
	} else { // Auto
		_target = _strategy.updateAuto(_rs, _sensors, nowMs);
	}

	// Apply motion controller (slew + heading hold)
	_out = _motion.compute(_rs, _sensors, _target, dt_s);

	// Final collision clamp (firmware-level)
	if (_sensors.distValid && _sensors.distMm <= cfg::DIST_EMERGENCY_STOP_MM) {
		_out.throttle = 0.0f;
	}
}

void App::writeActuators() {
	_motor.setThrottle(_out.throttle);
	_servo.setNormalized(_out.steer);
}

void App::sendTelemetry(uint32_t nowMs) {
	if (!_telemetryTimer.due(nowMs))
		return;

	proto::StatusPayload st{};
	st.now_ms = nowMs;
	st.mode = (uint8_t)_rs.mode;
	st.estop = (uint8_t)(_rs.estop == EstopState::Active ? 1 : 0);
	st.prefer_right = (uint8_t)(_rs.preferRight ? 1 : 0);

	st.dist_mm = _sensors.distValid ? _sensors.distMm : 0;
	st.yaw_cdeg =
		(int16_t)(mc::clamp(_sensors.yawDeg, -180.0f, 180.0f) * 100.0f);
	st.yawRate_cdeg_s =
		(int16_t)(mc::clamp(_sensors.yawRateDegS, -500.0f, 500.0f) * 100.0f);

	st.throttle_cmd_milli =
		(int16_t)(mc::clamp(_out.throttle, -1.0f, 1.0f) * 1000.0f);
	st.steer_cmd_milli =
		(int16_t)(mc::clamp(_out.steer, -1.0f, 1.0f) * 1000.0f);

	st.loop_hz = _loopHz;

	_commSerial.sendStatus(st);
	if (cfg::BLE_ENABLED) {
		_commBle.sendStatus(st);
	}
}

void App::loop() {
	const uint32_t now = millis();
	const float dt_s =
		(now > _lastLoopMs) ? ((now - _lastLoopMs) / 1000.0f) : 0.0f;
	_lastLoopMs = now;

	updateSensors();
	applyHostCommands(now);
	runStateMachine(now, dt_s);
	writeActuators();
	sendTelemetry(now);

	// loopHz estimation
	_loopCounter++;
	const uint32_t dt = now - _loopCounterTs;
	if (dt >= 1000) {
		_loopHz = (uint16_t)(_loopCounter * 1000UL / dt);
		_loopCounter = 0;
		_loopCounterTs = now;
	}
}

} // namespace mc
