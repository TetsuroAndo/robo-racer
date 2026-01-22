#include "mc_core/Log.h"
#include "mc_core/Time.h"
#include <inttypes.h>

namespace mc::log {

static const char *levelName(Level lv) {
	if (lv >= LV_FATAL)
		return "FATAL";
	if (lv >= LV_ERROR)
		return "ERROR";
	if (lv >= LV_WARN)
		return "WARN";
	if (lv >= LV_INFO)
		return "INFO";
	if (lv >= LV_DEBUG)
		return "DEBUG";
	return "TRACE";
}

void StdoutSink::write(const Record &r) {
	std::fprintf(stdout, "[%10" PRIu64 "us] %-5s %-20s %s\n", r.ts_us,
				 levelName(r.level), r.tag, r.msg);
	std::fflush(stdout);
}

FileSink::FileSink(const std::string &path) {
	fp_ = std::fopen(path.c_str(), "a");
}

FileSink::~FileSink() {
	if (fp_)
		std::fclose(fp_);
}

void FileSink::write(const Record &r) {
	if (!fp_)
		return;
	std::fprintf(fp_, "[%10" PRIu64 "us] %-5s %-20s %s\n", r.ts_us,
				 levelName(r.level), r.tag, r.msg);
	std::fflush(fp_);
}

AsyncLogger::AsyncLogger() : th_(&AsyncLogger::worker_, this) {}

AsyncLogger::~AsyncLogger() {
	running_.store(false);
	cv_.notify_all();
	if (th_.joinable())
		th_.join();
}

void AsyncLogger::addSink(std::unique_ptr< ISink > sink) {
	std::lock_guard< std::mutex > lk(sinks_m_);
	sinks_.push_back(std::move(sink));
}

void AsyncLogger::setMinLevel(Level lv) { min_level_.store(lv); }

bool AsyncLogger::tryPush_(const Record &r) {
	const uint32_t h = head_.load(std::memory_order_relaxed);
	const uint32_t t = tail_.load(std::memory_order_acquire);
	if ((h - t) >= QSIZE)
		return false;

	q_[h % QSIZE] = r;
	head_.store(h + 1, std::memory_order_release);
	return true;
}

bool AsyncLogger::log(Level lv, const char *tag, const char *msg) {
	if (lv < min_level_.load())
		return true;

	Record r{};
	r.ts_us = mc::Time::us();
	r.level = lv;
	std::snprintf(r.tag, sizeof(r.tag), "%s", tag ? tag : "");
	std::snprintf(r.msg, sizeof(r.msg), "%s", msg ? msg : "");

	if (!tryPush_(r)) {
		dropped_.fetch_add(1);
		return false;
	}
	cv_.notify_one();
	return true;
}

bool AsyncLogger::logf(Level lv, const char *tag, const char *fmt, ...) {
	if (lv < min_level_.load())
		return true;

	char buf[200];
	va_list ap;
	va_start(ap, fmt);
	std::vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);
	return log(lv, tag, buf);
}

void AsyncLogger::worker_() {
	while (running_.load()) {
		uint32_t t = tail_.load(std::memory_order_relaxed);
		uint32_t h = head_.load(std::memory_order_acquire);

		if (t == h) {
			std::unique_lock< std::mutex > lk(cv_m_);
			cv_.wait_for(lk, std::chrono::milliseconds(50));
			continue;
		}

		Record r = q_[t % QSIZE];
		tail_.store(t + 1, std::memory_order_release);

		std::lock_guard< std::mutex > lk(sinks_m_);
		for (auto &s : sinks_)
			s->write(r);
	}
}

} // namespace mc::log
