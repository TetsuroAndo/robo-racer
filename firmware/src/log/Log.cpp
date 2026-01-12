#include "log/Log.h"
#include <common/Time.h>
#include <stdio.h>

namespace mc::log {

Logger::Logger() { setLevelAll(Level::Info); }

Logger &Logger::instance() {
	static Logger logger;
	return logger;
}

void Logger::attachSink(Sink *sink) {
	if (!sink || _sinkCount >= kMaxSinks)
		return;
	_sinks[_sinkCount++] = sink;
}

void Logger::clearSinks() { _sinkCount = 0; }

void Logger::setLevelAll(Level level) {
	for (size_t i = 0; i < (size_t)Topic::Count; ++i)
		_levels[i] = level;
}

void Logger::setLevel(Topic topic, Level level) {
	const size_t idx = (size_t)topic;
	if (idx >= (size_t)Topic::Count)
		return;
	_levels[idx] = level;
}

bool Logger::enabled(Topic topic, Level level) const {
	const size_t idx = (size_t)topic;
	if (idx >= (size_t)Topic::Count)
		return false;
	return level >= _levels[idx];
}

void Logger::logf(Topic topic, Level level, const char *fmt, ...) {
	va_list args;
	va_start(args, fmt);
	vlogf(topic, level, fmt, args);
	va_end(args);
}

void Logger::vlogf(Topic topic, Level level, const char *fmt, va_list args) {
	if (!enabled(topic, level) || _sinkCount == 0)
		return;
	char message[128];
	vsnprintf(message, sizeof(message), fmt, args);
	Record rec{};
	rec.now_ms = Time::ms();
	rec.level = level;
	rec.topic = topic;
	for (size_t i = 0; i < _sinkCount; ++i) {
		if (_sinks[i])
			_sinks[i]->write(rec, message);
	}
}

void StreamSink::write(const Record &rec, const char *message) {
	_out.print(rec.now_ms);
	_out.print(" ");
	_out.print(levelToString(rec.level));
	_out.print(" ");
	_out.print(topicToString(rec.topic));
	_out.print(" ");
	_out.println(message);
}

const char *levelToString(Level level) {
	switch (level) {
	case Level::Trace:
		return "T";
	case Level::Debug:
		return "D";
	case Level::Info:
		return "I";
	case Level::Warn:
		return "W";
	case Level::Error:
		return "E";
	default:
		return "?";
	}
}

const char *topicToString(Topic topic) {
	switch (topic) {
	case Topic::App:
		return "APP";
	case Topic::Comm:
		return "COMM";
	case Topic::Control:
		return "CTRL";
	case Topic::Hardware:
		return "HW";
	case Topic::Hils:
		return "HILS";
	default:
		return "?";
	}
}

Level levelFromInt(uint8_t level) {
	const uint8_t max = static_cast< uint8_t >(Level::Off);
	if (level > max)
		return Level::Off;
	return static_cast< Level >(level);
}

} // namespace mc::log
