#include "InputSource.h"

#include "../config/Config.h"
#include "AutoCommandStore.h"
#include "ControllerInput.h"
#include "SafetyState.h"
#include <Arduino.h>

void InputSource::updateManual(ControllerInput &pad, SafetyState &safety) {
	pad.update();

	if (!pad.isConnected()) {
		manual_speed_ = 0;
		manual_angle_ = 0.0f;
		return;
	}

	const PadState &st = pad.state();
	if (st.home || st.start) {
		safety.kill_latched = true;
	}

	if (safety.auto_active) {
		manual_speed_ = 0;
		manual_angle_ = 0.0f;
		return;
	}

	if (st.dpad & 0x04) {
		manual_angle_ = cfg::MANUAL_STEER_DEG;
	} else if (st.dpad & 0x08) {
		manual_angle_ = -cfg::MANUAL_STEER_DEG;
	} else {
		manual_angle_ = 0.0f;
	}

	if (st.B) {
		manual_speed_ -= cfg::MANUAL_SPEED_STEP;
	} else if (st.A) {
		manual_speed_ += cfg::MANUAL_SPEED_STEP;
	} else {
		manual_speed_ -= cfg::MANUAL_SPEED_STEP;
	}

	manual_speed_ = constrain(manual_speed_, 0, cfg::MANUAL_SPEED_MAX);
}

ControlCommand InputSource::resolve(const SafetyState &safety,
									const AutoCommandStore &auto_cmd,
									bool hb_timeout, bool ttl_expired,
									bool &used_auto) const {
	used_auto = false;
	ControlCommand cmd{};

	if (safety.kill_latched) {
		return cmd;
	}

	if (safety.auto_active) {
		if (hb_timeout || ttl_expired) {
			return cmd;
		}
		used_auto = true;
		cmd.speed = auto_cmd.sp.speed_cmd;
		cmd.angle = static_cast< float >(auto_cmd.sp.steer_cdeg) /
					cfg::STEER_CDEG_SCALE;
		return cmd;
	}

	cmd.speed = manual_speed_;
	cmd.angle = manual_angle_;
	return cmd;
}
