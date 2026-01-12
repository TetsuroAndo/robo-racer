#include "comm/CommandMux.h"
#include "log/Log.h"

namespace mc::comm {

CommandMux::CommandMux(Stream &serial, BleUart *ble)
	: _serial(serial), _ble(ble) {}

void CommandMux::begin() { _bleConnected = _ble ? _ble->connected() : false; }

bool CommandMux::poll(RcCommand &out) {
	bool updated = false;
	if (_serialParser.poll(_serial, out)) {
		MC_LOGD(mc::log::Topic::Comm, "serial cmd: %.3f %.3f", out.throttle,
				out.steer);
		updated = true;
	}

	if (_ble) {
		_ble->poll();
		const bool connected = _ble->connected();
		if (connected != _bleConnected) {
			MC_LOGI(mc::log::Topic::Comm, "BLE %s",
					connected ? "connected" : "disconnected");
			_bleConnected = connected;
		}
		if (_bleParser.poll(*_ble, out)) {
			MC_LOGD(mc::log::Topic::Comm, "ble cmd: %.3f %.3f", out.throttle,
					out.steer);
			updated = true;
		}
	}

	return updated;
}

} // namespace mc::comm
