#include "AsyncLogger.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

namespace mc {

void AsyncLogger::begin(UartTx &tx) { _tx = &tx; }

bool AsyncLogger::enqueue_(const uint8_t *data, uint16_t len) {
	if (!_tx)
		return false;
	if (len == 0 || len > MSGMAX)
		return false;
	if (!_tx->enqueue(data, len)) {
		_dropped++;
		return false;
	}
	return true;
}

bool AsyncLogger::buildPayload_(LogLevel lv, const char *tag, const char *msg,
								uint8_t *payload_buf, uint16_t &payload_len) {
	payload_len = 0;
	if (!payload_buf)
		return false;

	payload_buf[0] = static_cast< uint8_t >(lv);

	static constexpr uint16_t MAX_TEXT = (uint16_t)(mc::proto::MAX_PAYLOAD - 1);

	char text[MAX_TEXT + 1];
	text[0] = '\0';

	const char *t = (tag && tag[0]) ? tag : nullptr;
	const char *m = (msg && msg[0]) ? msg : "";

	if (t) {
		snprintf(text, sizeof(text), "%s: %s", t, m);
	} else {
		snprintf(text, sizeof(text), "%s", m);
	}

	size_t n = strnlen(text, MAX_TEXT);
	memcpy(payload_buf + 1, text, n);

	payload_len = (uint16_t)(1 + n);
	return true;
}

bool AsyncLogger::buildFrame_(LogLevel lv, const char *tag, const char *msg,
							  uint8_t *frame_buf, uint16_t &frame_len) {
	frame_len = 0;
	if (!frame_buf)
		return false;

	uint8_t payload[mc::proto::MAX_PAYLOAD];
	uint16_t payload_len = 0;

	if (!buildPayload_(lv, tag, msg, payload, payload_len))
		return false;

	size_t out_len = 0;
	const bool ok = mc::proto::PacketWriter::build(
		frame_buf, MSGMAX, out_len, mc::proto::Type::LOG, 0, _seq++, payload,
		payload_len);
	if (!ok)
		return false;

	frame_len = (uint16_t)out_len;
	return true;
}

bool AsyncLogger::log(LogLevel lv, const char *tag, const char *msg) {
	if (!_tx)
		return false;
	if ((uint8_t)lv < (uint8_t)_min)
		return true;

	uint8_t frame[MSGMAX];
	uint16_t frame_len = 0;
	if (!buildFrame_(lv, tag, msg, frame, frame_len)) {
		_dropped++;
		return false;
	}
	return enqueue_(frame, frame_len);
}

bool AsyncLogger::logf(LogLevel lv, const char *tag, const char *fmt, ...) {
	if (!_tx)
		return false;
	if ((uint8_t)lv < (uint8_t)_min)
		return true;

	char buf[mc::proto::MAX_PAYLOAD];
	va_list ap;
	va_start(ap, fmt);
	vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);

	return log(lv, tag, buf);
}

} // namespace mc
