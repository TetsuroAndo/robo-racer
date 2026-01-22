#pragma once

#include <stdint.h>

class ControllerInput;
struct AutoCommandStore;
struct SafetyState;

struct ControlCommand {
	int speed = 0;
	float angle = 0.0f;
};

class InputSource {
public:
	void updateManual(ControllerInput &pad, SafetyState &safety);

	ControlCommand resolve(const SafetyState &safety,
						   const AutoCommandStore &auto_cmd, bool hb_timeout,
						   bool ttl_expired, bool &used_auto) const;

private:
	int manual_speed_ = 0;
	float manual_angle_ = 0.0f;
};
