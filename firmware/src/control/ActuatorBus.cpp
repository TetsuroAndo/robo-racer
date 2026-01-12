#include "control/ActuatorBus.h"

namespace mc {

ActuatorBus::ActuatorBus(int pin_rpwm, int pin_lpwm, int pin_ren, int pin_len,
						 int pin_servo)
	: _motor(pin_rpwm, pin_lpwm, pin_ren, pin_len), _servo(pin_servo) {}

void ActuatorBus::begin(uint16_t servoMinUs, uint16_t servoMaxUs,
						uint16_t servoCenterUs, uint32_t pwmFreqHz,
						uint8_t pwmResolutionBits) {
	_motor.begin(pwmFreqHz, pwmResolutionBits);
	_servo.begin(servoMinUs, servoMaxUs, servoCenterUs);
}

void ActuatorBus::setOutputMode(OutputMode mode) { _mode = mode; }

void ActuatorBus::attachHilsSink(hils::Sink *sink) { _hils = sink; }

void ActuatorBus::apply(float throttle, float steer, uint32_t now_ms) {
	_lastThrottle = throttle;
	_lastSteer = steer;

	if (_mode != OutputMode::Hils) {
		_motor.setThrottle(throttle);
		_servo.setNormalized(steer);
	}

	if ((_mode == OutputMode::Shadow || _mode == OutputMode::Hils) && _hils) {
		hils::ActuatorCommand cmd{};
		cmd.now_ms = now_ms;
		cmd.throttle = throttle;
		cmd.steer = steer;
		_hils->onActuatorCommand(cmd);
	}
}

} // namespace mc
