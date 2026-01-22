#include "Handlers.h"

#include "../protocol/Protocol.h"

void registerHandlers(Dispatcher &dispatcher) {
	dispatcher.reg(static_cast< uint8_t >(proto::Type::AUTO_MODE),
				   handleAutoMode);
	dispatcher.reg(static_cast< uint8_t >(proto::Type::AUTO_SETPOINT),
				   handleAutoSetpoint);
	dispatcher.reg(static_cast< uint8_t >(proto::Type::HEARTBEAT),
				   handleHeartbeat);
	dispatcher.reg(static_cast< uint8_t >(proto::Type::KILL), handleKill);
	dispatcher.reg(static_cast< uint8_t >(proto::Type::CLEAR_KILL),
				   handleClearKill);
}
