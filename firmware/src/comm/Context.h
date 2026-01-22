#pragma once

#include "../comm/TxPort.h"
#include "../control/AutoCommandStore.h"
#include "../control/SafetyState.h"
#include "../hardware/Drive.h"

struct Context {
	Drive &drive;
	SafetyState &safety;
	AutoCommandStore &autoCmd;
	TxPort &tx;
};
