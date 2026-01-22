#include "async_logger.h"
#include <chrono>

void AsyncLogger::addSink(std::unique_ptr< ILogSink > s) {
	_sinks.emplace_back(std::move(s));
}

uint64_t AsyncLogger::nowNs_() {
	using namespace std::chrono;
	return (uint64_t)duration_cast< nanoseconds >(
			   steady_clock::now().time_since_epoch())
		.count();
}

void AsyncLogger::start() {
	_run = true;
	_th = std::thread([this] { loop_(); });
}

void AsyncLogger::stop() {
	_run = false;
	_cv.notify_all();
	if (_th.joinable())
		_th.join();
}

void AsyncLogger::log(LogLevel lv, const std::string &msg) {
	if (!_run)
		return;
	{
		std::lock_guard< std::mutex > lk(_mu);
		_q.push(LogEvent{nowNs_(), lv, msg});
	}
	_cv.notify_one();
}

void AsyncLogger::loop_() {
	while (_run) {
		LogEvent e{};
		{
			std::unique_lock< std::mutex > lk(_mu);
			_cv.wait(lk, [&] { return !_q.empty() || !_run; });
			if (!_run && _q.empty())
				break;
			e = std::move(_q.front());
			_q.pop();
		}
		for (auto &s : _sinks)
			s->write(e);
	}
}
