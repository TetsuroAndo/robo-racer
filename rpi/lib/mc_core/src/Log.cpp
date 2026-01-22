#include <mc/core/Log.hpp>
#include <mc/core/Time.hpp>

#include <cstdio>
#include <fstream>

namespace mc::core {

static const char *level_str(LogLevel lv) {
	switch (lv) {
	case LogLevel::Trace:
		return "TRACE";
	case LogLevel::Debug:
		return "DEBUG";
	case LogLevel::Info:
		return "INFO";
	case LogLevel::Warn:
		return "WARN";
	case LogLevel::Error:
		return "ERROR";
	case LogLevel::Fatal:
		return "FATAL";
	}
	return "UNK";
}

void ConsoleSink::write(const LogRecord &r) {
	std::fprintf(stderr, "[%llu] %-5s [%s] %s\n", (unsigned long long)r.ts_us,
				 level_str(r.level), r.tag.c_str(), r.msg.c_str());
}

FileSink::FileSink(std::string path) : path_(std::move(path)) {}

void FileSink::write(const LogRecord &r) {
	std::ofstream ofs(path_, std::ios::app);
	ofs << "[" << r.ts_us << "] " << level_str(r.level) << " [" << r.tag << "] "
		<< r.msg << "\n";
}

Logger &Logger::instance() {
	static Logger g;
	return g;
}

Logger::Logger() : th_([this] { worker_(); }) {
	sinks_.push_back(std::make_shared< ConsoleSink >());
}

Logger::~Logger() { shutdown(); }

void Logger::addSink(std::shared_ptr< ILogSink > sink) {
	std::lock_guard< std::mutex > lk(mtx_);
	sinks_.push_back(std::move(sink));
}

void Logger::log(LogLevel lv, std::string tag, std::string msg) {
	if ((uint8_t)lv < level_.load())
		return;

	LogRecord r;
	r.ts_us = now_us_monotonic();
	r.level = lv;
	r.tag = std::move(tag);
	r.msg = std::move(msg);

	{
		std::lock_guard< std::mutex > lk(mtx_);
		constexpr size_t MAX_Q = 4096;
		if (q_.size() >= MAX_Q)
			q_.pop_front();
		q_.push_back(std::move(r));
	}
	cv_.notify_one();
}

void Logger::shutdown() {
	bool expected = true;
	if (!running_.compare_exchange_strong(expected, false))
		return;
	cv_.notify_one();
	if (th_.joinable())
		th_.join();
}

void Logger::worker_() {
	while (running_.load()) {
		std::deque< LogRecord > local;
		{
			std::unique_lock< std::mutex > lk(mtx_);
			cv_.wait(lk, [&] { return !running_.load() || !q_.empty(); });
			local.swap(q_);
		}
		for (auto &r : local) {
			std::vector< std::shared_ptr< ILogSink > > sinks_copy;
			{
				std::lock_guard< std::mutex > lk(mtx_);
				sinks_copy = sinks_;
			}
			for (auto &s : sinks_copy)
				s->write(r);
		}
	}

	std::deque< LogRecord > local;
	{
		std::lock_guard< std::mutex > lk(mtx_);
		local.swap(q_);
	}
	for (auto &r : local) {
		for (auto &s : sinks_)
			s->write(r);
	}
}

} // namespace mc::core
