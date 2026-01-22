#pragma once
#include <Arduino.h>
#include <stdint.h>

namespace mc {

class UartTx {
public:
	void begin(HardwareSerial& uart);

	bool enqueue(const uint8_t* data, uint16_t len);

	uint32_t dropped() const { return _dropped; }

private:
	HardwareSerial* _uart = nullptr;

	static constexpr int QDEPTH = 32;
	static constexpr size_t MSGMAX = 256;

	struct TxMsg {
		uint16_t len;
		uint8_t data[MSGMAX];
	};

	StaticQueue_t _qbuf{};
	uint8_t _qstorage[QDEPTH * sizeof(TxMsg)]{};
	QueueHandle_t _q = nullptr;

	TaskHandle_t _task = nullptr;

	uint32_t _dropped = 0;

	static void txTask_(void* arg);
	void txLoop_();
};

} // namespace mc
