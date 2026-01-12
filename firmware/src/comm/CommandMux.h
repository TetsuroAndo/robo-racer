#pragma once
#include <Stream.h>
#include "comm/BleUart.h"
#include "comm/Command.h"
#include "comm/LineProtocol.h"

namespace mc::comm {

class CommandMux {
 public:
	CommandMux(Stream &serial, BleUart *ble);

	void begin();
	bool poll(RcCommand &out);

 private:
	Stream &_serial;
	BleUart *_ble = nullptr;
	LineProtocol _serialParser{};
	LineProtocol _bleParser{};
	bool _bleConnected = false;
};

} // namespace mc::comm
