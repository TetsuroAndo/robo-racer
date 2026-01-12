#pragma once
#include <stdint.h>
#include "hardware/MotorDriver.h"
#include "hardware/ServoSteering.h"
#include "hils/Hils.h"

namespace mc {

enum class OutputMode : uint8_t {
	Hardware = 0,
	Shadow = 1,
	Hils = 2,
};

class ActuatorBus {
 public:
	ActuatorBus(int pin_rpwm, int pin_lpwm, int pin_ren, int pin_len,
				int pin_servo);

	void begin(uint16_t servoMinUs, uint16_t servoMaxUs, uint16_t servoCenterUs,
			   uint32_t pwmFreqHz = 20000, uint8_t pwmResolutionBits = 10);

	void setOutputMode(OutputMode mode);
	OutputMode outputMode() const { return _mode; }

	void attachHilsSink(hils::Sink *sink);
	void apply(float throttle, float steer, uint32_t now_ms);

	float lastThrottle() const { return _lastThrottle; }
	float lastSteer() const { return _lastSteer; }

 private:
	MotorDriver _motor;
	ServoSteering _servo;
	OutputMode _mode = OutputMode::Hardware;
	hils::Sink *_hils = nullptr;
	float _lastThrottle = 0.0f;
	float _lastSteer = 0.0f;
};

} // namespace mc
