#include "control/MotionController.h"

namespace mc {

MotionController::MotionController(float yawKp, float throttleMax, float rateUp,
								   float rateDn)
	: _yawKp(yawKp), _throttleMax(throttleMax),
	  _throttleLimiter(rateUp, rateDn, 0.0f) {}

void MotionController::reset() { _throttleLimiter.reset(0.0f); }

ControlCommand MotionController::compute(const RuntimeState &rs,
										 const SensorData &s,
										 const ControlCommand &target,
										 float dt_s) {

	ControlCommand out = target;
	dt_s = (dt_s <= 0.0f) ? _lastDt : dt_s;
	_lastDt = dt_s;

	// Clamp and slew throttle
	out.throttle = mc::clamp(out.throttle, -1.0f, 1.0f);
	out.throttle = mc::clamp(out.throttle, -_throttleMax, _throttleMax);
	out.throttle = _throttleLimiter.update(out.throttle, dt_s);

	// Heading hold adds steering correction (if IMU is valid)
	if (rs.hasHeadingRef && s.imuValid) {
		const float err = mc::wrapDeg180(rs.headingRefDeg - s.yawDeg);
		out.steer += mc::clamp(_yawKp * err, -0.8f, 0.8f);
	}

	out.steer = mc::clamp(out.steer, -1.0f, 1.0f);
	return out;
}

} // namespace mc
