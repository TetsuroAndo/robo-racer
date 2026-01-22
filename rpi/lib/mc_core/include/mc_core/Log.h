#pragma once
#include <atomic>
#include <cstdarg>
#include <cstdint>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace mc::log {

using Level = uint8_t;

static constexpr Level LV_TRACE = 10;
static constexpr Level LV_DEBUG = 20;
static constexpr Level LV_INFO  = 30;
static constexpr Level LV_WARN  = 40;
static constexpr Level LV_ERROR = 50;
static constexpr Level LV_FATAL = 60;

struct Record {
	uint64_t ts_us;
	Level level;
	char tag[24];
	char msg[200];
};

class ISink {
public:
	virtual ~ISink() = default;
	virtual void write(const Record& r) = 0;
};

class StdoutSink final : public ISink {
public:
	void write(const Record& r) override;
};

class FileSink final : public ISink {
public:
	explicit FileSink(const std::string& path);
	~FileSink() override;
	void write(const Record& r) override;
private:
	FILE* fp_ = nullptr;
};

class AsyncLogger {
public:
	AsyncLogger();
	~AsyncLogger();

	void addSink(std::unique_ptr<ISink> sink);
	void setMinLevel(Level lv);

	bool log(Level lv, const char* tag, const char* msg);
	bool logf(Level lv, const char* tag, const char* fmt, ...);

	uint64_t dropped() const { return dropped_.load(); }

private:
	static constexpr uint32_t QSIZE = 1024;

	std::atomic<uint32_t> head_{0};
	std::atomic<uint32_t> tail_{0};
	std::atomic<uint64_t> dropped_{0};
	std::atomic<Level> min_level_{LV_INFO};

	Record q_[QSIZE];

	std::mutex cv_m_;
	std::condition_variable cv_;
	std::atomic<bool> running_{true};
	std::thread th_;

	std::mutex sinks_m_;
	std::vector<std::unique_ptr<ISink>> sinks_;

	void worker_();
	bool tryPush_(const Record& r);
};

} // namespace mc::log
