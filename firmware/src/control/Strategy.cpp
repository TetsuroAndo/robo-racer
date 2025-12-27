#include "control/Strategy.h"
#include "config/Config.h"
#include <common/Math.h>

namespace mc {

Strategy::Strategy() {}

bool Strategy::obstacleClose(const SensorData &s, uint16_t thrMm) const {
	return s.distValid && s.distMm <= thrMm;
}

void Strategy::enter(RuntimeState &rs, AutoSubstate st, uint32_t nowMs,
					 const SensorData &s) {
	_sub = st;
	_subStartMs = nowMs;
	rs.maneuverStartMs = nowMs;
	rs.maneuverYawStartDeg = s.imuValid ? s.yawDeg : 0.0f;
}

ControlCommand Strategy::updateAuto(RuntimeState &rs, const SensorData &s,
									uint32_t nowMs) {
	ControlCommand cmd{};

	// Default heading reference management:
	// - When cruising, keep initial heading and correct drift using gyro.
	// - When we start a maneuver, disable heading hold until cruise resumes.
	if (_sub == AutoSubstate::Cruise) {
		if (!rs.hasHeadingRef && s.imuValid) {
			rs.hasHeadingRef = true;
			rs.headingRefDeg = s.yawDeg;
		}
	}

	// Emergency: immediate stop and reverse behavior is in state machine, not
	// here.
	const bool mustStop = obstacleClose(s, cfg::DIST_EMERGENCY_STOP_MM);

	switch (_sub) {
	case AutoSubstate::Cruise: {
		if (mustStop) {
			rs.hasHeadingRef = false;
			enter(rs, AutoSubstate::Brake, nowMs, s);
			cmd.throttle = 0.0f;
			cmd.steer = 0.0f;
			break;
		}

		// Speed profile based on distance
		if (obstacleClose(s, cfg::DIST_SLOW_MM)) {
			cmd.throttle = cfg::THROTTLE_MAX * 0.45f;
		} else {
			cmd.throttle = cfg::THROTTLE_MAX;
		}
		cmd.steer = 0.0f;
		break;
	}
	case AutoSubstate::Brake: {
		cmd.throttle = 0.0f;
		cmd.steer = 0.0f;
		if ((nowMs - _subStartMs) >= cfg::TURN_BRAKE_MS) {
			enter(rs, AutoSubstate::Reverse, nowMs, s);
		}
		break;
	}
	case AutoSubstate::Reverse: {
		cmd.throttle = cfg::THROTTLE_REVERSE;
		cmd.steer = 0.0f;
		if ((nowMs - _subStartMs) >= cfg::TURN_REVERSE_MS) {
			enter(rs, AutoSubstate::Turn, nowMs, s);
		}
		break;
	}
	case AutoSubstate::Turn: {
		// Prefer right/left rule.
		const float steerDir = rs.preferRight ? +1.0f : -1.0f;
		cmd.throttle = cfg::TURN_THROTTLE;
		cmd.steer = steerDir;

		if (s.imuValid) {
			const float dYaw =
				mc::wrapDeg180(s.yawDeg - rs.maneuverYawStartDeg);
			if (fabsf(dYaw) >= cfg::TURN_TARGET_DEG) {
				// Finish maneuver: back to cruise and re-seed heading reference
				rs.hasHeadingRef = true;
				rs.headingRefDeg = s.yawDeg;
				enter(rs, AutoSubstate::Cruise, nowMs, s);
				cmd.throttle = 0.0f;
				cmd.steer = 0.0f;
			}
		} else {
			// If no IMU, fallback to a time-based turn.
			if ((nowMs - _subStartMs) >= 700) {
				enter(rs, AutoSubstate::Cruise, nowMs, s);
			}
		}
		break;
	}
	}

	// If sensor says very close obstacle, force throttle 0 to avoid ramming.
	if (mustStop)
		cmd.throttle = 0.0f;

	return cmd;
}

} // namespace mc
