#pragma once
#include <Arduino.h>
#include <stdint.h>
#include "../comm/mc_proto.h"

namespace mc {

enum class LogLevel : uint8_t { TRACE=0, DEBUG=1, INFO=2, WARN=3, ERROR=4, FATAL=5 };

#pragma pack(push, 1)
struct LogPayload {
	uint8_t level;
	uint8_t category;
	uint16_t code_le;
	uint32_t t_ms_le;
	int32_t a0_le;
	int32_t a1_le;
	int32_t a2_le;
	int32_t a3_le;
};
#pragma pack(pop)

class AsyncLogger {
public:
	void begin(HardwareSerial& uart);

	void log(LogLevel lv, uint8_t cat, uint16_t code,
			 int32_t a0=0, int32_t a1=0, int32_t a2=0, int32_t a3=0);

	void setMinLevel(LogLevel lv) { _min = lv; }

private:
	HardwareSerial* _uart = nullptr;
	LogLevel _min = LogLevel::INFO;

	static constexpr int QDEPTH = 16;
	static constexpr size_t MSGMAX = mc::proto::MAX_FRAME_ENCODED;

	struct TxMsg { uint16_t len; uint8_t data[MSGMAX]; };

	StaticQueue_t _qbuf;
	uint8_t _qstorage[QDEPTH * sizeof(TxMsg)];
	QueueHandle_t _q = nullptr;

	TaskHandle_t _task = nullptr;

	static void txTask_(void* arg);
	void txLoop_();
	bool enqueue_(const uint8_t* data, uint16_t len);
};

} // namespace mc
