#include "AsyncLogger.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>

namespace mc {

static inline void wr16(uint8_t *p, uint16_t v) {
	p[0] = (uint8_t)(v & 0xFF);
	p[1] = (uint8_t)(v >> 8);
}
static inline void wr32(uint8_t *p, uint32_t v) {
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
	p[2] = (uint8_t)(v >> 16);
	p[3] = (uint8_t)(v >> 24);
}
static inline void wr32s(uint8_t *p, int32_t v) { wr32(p, (uint32_t)v); }

void AsyncLogger::begin(HardwareSerial &uart) {
	_uart = &uart;
	if (!_q) {
		_q = xQueueCreateStatic(QDEPTH, sizeof(TxMsg), _qstorage, &_qbuf);
	}
	if (!_task) {
		xTaskCreatePinnedToCore(&AsyncLogger::txTask_, "mc_log_tx", 4096, this,
								1, &_task, 0);
	}
}

bool AsyncLogger::enqueue_(const uint8_t *data, uint16_t len) {
	if (!_q || len == 0 || len > MSGMAX)
		return false;
	TxMsg m{};
	m.len = len;
	memcpy(m.data, data, len);
	return xQueueSend(_q, &m, 0) == pdTRUE;
}

void AsyncLogger::log(LogLevel lv, uint8_t cat, uint16_t code, int32_t a0,
					  int32_t a1, int32_t a2, int32_t a3) {
	if (!_uart)
		return;
	if ((uint8_t)lv < (uint8_t)_min)
		return;

	LogPayload p{};
	p.level = (uint8_t)lv;
	p.category = cat;
	wr16((uint8_t *)&p.code_le, code);
	wr32((uint8_t *)&p.t_ms_le, (uint32_t)millis());
	wr32s((uint8_t *)&p.a0_le, a0);
	wr32s((uint8_t *)&p.a1_le, a1);
	wr32s((uint8_t *)&p.a2_le, a2);
	wr32s((uint8_t *)&p.a3_le, a3);

	uint8_t frame[mc::proto::MAX_FRAME_ENCODED];
	size_t out_len = 0;
	if (!mc::proto::PacketWriter::build(
			frame, sizeof(frame), out_len, mc::proto::Type::LOG, 0, 0,
			(const uint8_t *)&p, (uint16_t)sizeof(p))) {
		return;
	}
	enqueue_(frame, (uint16_t)out_len);
}

void AsyncLogger::txTask_(void *arg) {
	static_cast< AsyncLogger * >(arg)->txLoop_();
}

void AsyncLogger::txLoop_() {
	while (true) {
		TxMsg m{};
		if (xQueueReceive(_q, &m, portMAX_DELAY) == pdTRUE) {
			if (_uart && m.len) {
				_uart->write(m.data, m.len);
			}
		}
	}
}

} // namespace mc
