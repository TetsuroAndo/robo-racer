#include "hils/Hils.h"

namespace mc::hils {

void Mux::attach(Sink *sink) {
	if (!sink || _count >= kMaxSinks)
		return;
	_sinks[_count++] = sink;
}

void Mux::clear() { _count = 0; }

void Mux::onActuatorCommand(const ActuatorCommand &cmd) {
	for (size_t i = 0; i < _count; ++i) {
		if (_sinks[i])
			_sinks[i]->onActuatorCommand(cmd);
	}
}

void StreamSink::onActuatorCommand(const ActuatorCommand &cmd) {
	_out.print("HILS ");
	_out.print(cmd.now_ms);
	_out.print(" ");
	_out.print(cmd.throttle, 3);
	_out.print(" ");
	_out.println(cmd.steer, 3);
}

} // namespace mc::hils
