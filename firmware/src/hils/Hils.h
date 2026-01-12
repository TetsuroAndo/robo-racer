#pragma once
#include <Arduino.h>
#include <stdint.h>

namespace mc::hils {

struct ActuatorCommand {
	uint32_t now_ms = 0;
	float throttle = 0.0f;
	float steer = 0.0f;
};

class Sink {
 public:
	virtual ~Sink() = default;
	virtual void onActuatorCommand(const ActuatorCommand &cmd) = 0;
};

class Mux : public Sink {
 public:
	void attach(Sink *sink);
	void clear();
	void onActuatorCommand(const ActuatorCommand &cmd) override;

 private:
	static constexpr size_t kMaxSinks = 3;
	Sink *_sinks[kMaxSinks]{};
	size_t _count = 0;
};

class StreamSink : public Sink {
 public:
	explicit StreamSink(Print &out) : _out(out) {}
	void onActuatorCommand(const ActuatorCommand &cmd) override;

 private:
	Print &_out;
};

} // namespace mc::hils
