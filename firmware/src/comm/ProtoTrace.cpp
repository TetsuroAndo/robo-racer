#include "ProtoTrace.h"

#include "../config/Config.h"

namespace {
constexpr const char *kNone = "none";
}

const char *ProtoTrace::errorName_(RxError err) const {
	switch (err) {
	case RxError::kOverflow:
		return "overflow";
	case RxError::kDecode:
		return "decode";
	case RxError::kTooShort:
		return "too_short";
	case RxError::kBadLength:
		return "bad_len";
	case RxError::kCrcMismatch:
		return "crc";
	case RxError::kBadVersion:
		return "bad_ver";
	case RxError::kUnsupported:
		return "unsupported";
	default:
		return "unknown";
	}
}

void ProtoTrace::onRxFrame(uint8_t type, uint8_t seq, uint16_t len,
						   uint8_t flags) {
	if (!cfg::PROTO_TRACE_ENABLE) {
		return;
	}
	++rx_ok_;
	last_rx_type_ = type;
	last_rx_seq_ = seq;

	if (cfg::PROTO_TRACE_VERBOSE) {
		Serial.printf(
			"[PROTO] RX_OK type=0x%02X seq=0x%02X len=%u flags=0x%02X\n", type,
			seq, static_cast< unsigned int >(len), flags);
	}
}

void ProtoTrace::onRxError(RxError err) {
	if (!cfg::PROTO_TRACE_ENABLE) {
		return;
	}
	switch (err) {
	case RxError::kOverflow:
		++rx_overflow_;
		break;
	case RxError::kDecode:
		++rx_decode_;
		break;
	case RxError::kTooShort:
		++rx_too_short_;
		break;
	case RxError::kBadLength:
		++rx_bad_len_;
		break;
	case RxError::kCrcMismatch:
		++rx_crc_;
		break;
	case RxError::kBadVersion:
		++rx_bad_ver_;
		break;
	case RxError::kUnsupported:
		++rx_unsupported_;
		break;
	}
	last_err_ = err;
	has_err_ = true;

	if (cfg::PROTO_TRACE_VERBOSE) {
		Serial.printf("[PROTO] RX_ERR err=%s\n", errorName_(err));
	}
}

void ProtoTrace::onTx(uint8_t type, uint8_t seq, uint16_t len, uint8_t flags,
					  bool ok) {
	if (!cfg::PROTO_TRACE_ENABLE) {
		return;
	}
	if (ok) {
		++tx_ok_;
	} else {
		++tx_fail_;
	}
	last_tx_type_ = type;
	last_tx_seq_ = seq;

	if (cfg::PROTO_TRACE_VERBOSE) {
		Serial.printf(
			"[PROTO] TX_%s type=0x%02X seq=0x%02X len=%u flags=0x%02X\n",
			ok ? "OK" : "FAIL", type, seq, static_cast< unsigned int >(len),
			flags);
	}
}

void ProtoTrace::tick(uint32_t now_ms) {
	if (!cfg::PROTO_TRACE_ENABLE) {
		return;
	}
	if ((uint32_t)(now_ms - last_summary_ms_) < cfg::PROTO_TRACE_SUMMARY_MS) {
		return;
	}
	last_summary_ms_ = now_ms;
	logSummary_();
}

void ProtoTrace::logSummary_() const {
	const char *err_name = has_err_ ? errorName_(last_err_) : kNone;
	Serial.printf(
		"[PROTO] rx_ok=%lu rx_crc=%lu rx_len=%lu rx_dec=%lu rx_ovf=%lu "
		"rx_ver=%lu rx_unsup=%lu tx_ok=%lu tx_fail=%lu "
		"last_rx=0x%02X/0x%02X last_tx=0x%02X/0x%02X last_err=%s\n",
		static_cast< unsigned long >(rx_ok_),
		static_cast< unsigned long >(rx_crc_),
		static_cast< unsigned long >(rx_bad_len_),
		static_cast< unsigned long >(rx_decode_),
		static_cast< unsigned long >(rx_overflow_),
		static_cast< unsigned long >(rx_bad_ver_),
		static_cast< unsigned long >(rx_unsupported_),
		static_cast< unsigned long >(tx_ok_),
		static_cast< unsigned long >(tx_fail_), last_rx_type_, last_rx_seq_,
		last_tx_type_, last_tx_seq_, err_name);
}
