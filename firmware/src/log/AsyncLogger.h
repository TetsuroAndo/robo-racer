#pragma once
#include <Arduino.h>
#include <stdint.h>
#include "../comm/mc_proto.h"
#include "../comm/UartTx.h"

namespace mc {

enum class LogLevel : uint8_t {
	TRACE = 0,
	DEBUG = 1,
	INFO = 2,
	WARN = 3,
	ERROR = 4,
	FATAL = 5
};

class AsyncLogger {
public:
	void begin(UartTx& tx);

	void setMinLevel(LogLevel lv) { _min = lv; }
	LogLevel minLevel() const { return _min; }

	bool log(LogLevel lv, const char* tag, const char* msg);
	bool logf(LogLevel lv, const char* tag, const char* fmt, ...);

	uint32_t dropped() const { return _dropped; }

private:
	UartTx* _tx = nullptr;
	LogLevel _min = LogLevel::INFO;

	static constexpr size_t MSGMAX = mc::proto::MAX_FRAME_ENCODED;

	uint16_t _seq = 1;
	uint32_t _dropped = 0;

	bool enqueue_(const uint8_t* data, uint16_t len);

	bool buildPayload_(LogLevel lv, const char* tag, const char* msg,
		uint8_t* payload_buf, uint16_t& payload_len);
	bool buildFrame_(LogLevel lv, const char* tag, const char* msg,
		uint8_t* frame_buf, uint16_t& frame_len);
};

} // namespace mc
