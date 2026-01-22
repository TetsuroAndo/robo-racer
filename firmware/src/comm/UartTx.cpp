#include "UartTx.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>

namespace mc {

void UartTx::begin(HardwareSerial &uart) {
	_uart = &uart;

	if (!_q) {
		_q = xQueueCreateStatic(QDEPTH, sizeof(TxMsg), _qstorage, &_qbuf);
	}
	if (!_task) {
		xTaskCreatePinnedToCore(&UartTx::txTask_, "mc_uart_tx", 4096, this, 1,
								&_task, 0);
	}
}

bool UartTx::enqueue(const uint8_t *data, uint16_t len) {
	if (!_q || !_uart)
		return false;
	if (!data || len == 0 || len > MSGMAX)
		return false;

	TxMsg m{};
	m.len = len;
	memcpy(m.data, data, len);

	if (xQueueSend(_q, &m, 0) != pdTRUE) {
		_dropped++;
		return false;
	}
	return true;
}

void UartTx::txTask_(void *arg) { static_cast< UartTx * >(arg)->txLoop_(); }

void UartTx::txLoop_() {
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
