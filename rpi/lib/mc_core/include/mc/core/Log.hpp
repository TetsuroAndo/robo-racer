#pragma once
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace mc::core {

enum class LogLevel : uint8_t { Trace = 0, Debug, Info, Warn, Error, Fatal };

struct LogRecord {
	uint64_t ts_us{};
	LogLevel level{};
	std::string tag;
	std::string msg;
};

struct ILogSink {
	virtual ~ILogSink() = default;
	virtual void write(const LogRecord &r) = 0;
};

class ConsoleSink final : public ILogSink {
public:
	void write(const LogRecord &r) override;
};

class FileSink final : public ILogSink {
public:
	explicit FileSink(std::string path);
	void write(const LogRecord &r) override;

private:
	std::string path_;
};

class Logger {
public:
	static Logger &instance();

	void addSink(std::shared_ptr< ILogSink > sink);
	void setConsoleEnabled(bool enabled);

	void setLevel(LogLevel lv) { level_.store((uint8_t)lv); }
	LogLevel level() const { return (LogLevel)level_.load(); }

	void log(LogLevel lv, std::string tag, std::string msg);
	void shutdown();

private:
	Logger();
	~Logger();

	void worker_();

	std::atomic< uint8_t > level_{(uint8_t)LogLevel::Info};
	std::atomic< bool > running_{true};

	std::mutex mtx_;
	std::condition_variable cv_;
	std::deque< LogRecord > q_;
	std::vector< std::shared_ptr< ILogSink > > sinks_;
	std::shared_ptr< ConsoleSink > console_sink_;
	std::atomic< bool > console_enabled_{true};

	std::thread th_;
};

} // namespace mc::core

#define MC_LOGT(tag, msg)                                                      \
	::mc::core::Logger::instance().log(::mc::core::LogLevel::Trace, (tag),     \
									   (msg))
#define MC_LOGD(tag, msg)                                                      \
	::mc::core::Logger::instance().log(::mc::core::LogLevel::Debug, (tag),     \
									   (msg))
#define MC_LOGI(tag, msg)                                                      \
	::mc::core::Logger::instance().log(::mc::core::LogLevel::Info, (tag), (msg))
#define MC_LOGW(tag, msg)                                                      \
	::mc::core::Logger::instance().log(::mc::core::LogLevel::Warn, (tag), (msg))
#define MC_LOGE(tag, msg)                                                      \
	::mc::core::Logger::instance().log(::mc::core::LogLevel::Error, (tag),     \
									   (msg))
#define MC_LOGF(tag, msg)                                                      \
	::mc::core::Logger::instance().log(::mc::core::LogLevel::Fatal, (tag),     \
									   (msg))
