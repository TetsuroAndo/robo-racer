#pragma once
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>
#include <memory>
#include <stdint.h>

enum class LogLevel : uint8_t { TRACE=0, DEBUG=1, INFO=2, WARN=3, ERROR=4, FATAL=5 };

struct LogEvent {
	uint64_t ts_ns;
	LogLevel level;
	std::string text;
};

class ILogSink {
public:
	virtual ~ILogSink() {}
	virtual void write(const LogEvent& e) = 0;
};

class AsyncLogger {
public:
	void addSink(std::unique_ptr<ILogSink> s);
	void start();
	void stop();

	void log(LogLevel lv, const std::string& msg);

private:
	std::vector<std::unique_ptr<ILogSink>> _sinks;

	std::mutex _mu;
	std::condition_variable _cv;
	std::queue<LogEvent> _q;
	std::atomic<bool> _run{false};
	std::thread _th;

	void loop_();
	static uint64_t nowNs_();
};
