#pragma once
#include <Arduino.h>
#include <cstdarg>
#include <cstdint>

namespace mc::log {

enum class Level : uint8_t {
	Trace = 0,
	Debug = 1,
	Info = 2,
	Warn = 3,
	Error = 4,
	Off = 5,
};

enum class Topic : uint8_t {
	App = 0,
	Comm = 1,
	Control = 2,
	Hardware = 3,
	Hils = 4,
	Count
};

struct Record {
	uint32_t now_ms = 0;
	Level level = Level::Info;
	Topic topic = Topic::App;
};

class Sink {
 public:
	virtual ~Sink() = default;
	virtual void write(const Record &rec, const char *message) = 0;
};

class StreamSink : public Sink {
 public:
	explicit StreamSink(Print &out) : _out(out) {}
	void write(const Record &rec, const char *message) override;

 private:
	Print &_out;
};

class Logger {
 public:
	static Logger &instance();

	void attachSink(Sink *sink);
	void clearSinks();

	void setLevelAll(Level level);
	void setLevel(Topic topic, Level level);

	bool enabled(Topic topic, Level level) const;
	void logf(Topic topic, Level level, const char *fmt, ...);
	void vlogf(Topic topic, Level level, const char *fmt, va_list args);

 private:
	Logger();

	static constexpr size_t kMaxSinks = 3;
	Sink *_sinks[kMaxSinks]{};
	size_t _sinkCount = 0;
	Level _levels[(size_t)Topic::Count]{};
};

const char *levelToString(Level level);
const char *topicToString(Topic topic);
Level levelFromInt(uint8_t level);

} // namespace mc::log

#define MC_LOGT(topic, fmt, ...)                                            \
	do {                                                                    \
		if (mc::log::Logger::instance().enabled(topic,                        \
												mc::log::Level::Trace))      \
			mc::log::Logger::instance().logf(                                 \
				topic, mc::log::Level::Trace, fmt, ##__VA_ARGS__);             \
	} while (0)

#define MC_LOGD(topic, fmt, ...)                                            \
	do {                                                                    \
		if (mc::log::Logger::instance().enabled(topic,                        \
												mc::log::Level::Debug))      \
			mc::log::Logger::instance().logf(                                 \
				topic, mc::log::Level::Debug, fmt, ##__VA_ARGS__);             \
	} while (0)

#define MC_LOGI(topic, fmt, ...)                                            \
	do {                                                                    \
		if (mc::log::Logger::instance().enabled(topic,                        \
												mc::log::Level::Info))       \
			mc::log::Logger::instance().logf(                                 \
				topic, mc::log::Level::Info, fmt, ##__VA_ARGS__);              \
	} while (0)

#define MC_LOGW(topic, fmt, ...)                                            \
	do {                                                                    \
		if (mc::log::Logger::instance().enabled(topic,                        \
												mc::log::Level::Warn))       \
			mc::log::Logger::instance().logf(                                 \
				topic, mc::log::Level::Warn, fmt, ##__VA_ARGS__);              \
	} while (0)

#define MC_LOGE(topic, fmt, ...)                                            \
	do {                                                                    \
		if (mc::log::Logger::instance().enabled(topic,                        \
												mc::log::Level::Error))      \
			mc::log::Logger::instance().logf(                                 \
				topic, mc::log::Level::Error, fmt, ##__VA_ARGS__);             \
	} while (0)
