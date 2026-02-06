#pragma once

#include "Targets.h"
#include "../comm/registry.h"
#include <stdint.h>

struct AutoCommandResult {
	Targets targets{};
	bool fresh = false;
};

class AutoCommandSource {
public:
	AutoCommandResult update(uint32_t now_ms, const mc::ControlState &st) const;
};
