#include "Trace.h"

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace proto {
namespace {
constexpr const char *kNone = "none";
constexpr const char *kEnvVerbose = "PROTO_TRACE_VERBOSE";
constexpr const char *kEnvLogfile = "PROTO_TRACE_LOGFILE";
constexpr const char *kEnvInterval = "PROTO_TRACE_SUMMARY_MS";

bool envTruth(const char *value) {
	if (!value) {
		return false;
	}
	return std::strcmp(value, "1") == 0 || std::strcmp(value, "true") == 0 ||
		   std::strcmp(value, "TRUE") == 0;
}
} // namespace

Trace::Trace() { configureFromEnv_(); }

Trace::~Trace() {
	if (out_ && out_ != stderr) {
		std::fclose(out_);
	}
}

void Trace::configureFromEnv_() {
	verbose_ = envTruth(std::getenv(kEnvVerbose));
	const char *interval = std::getenv(kEnvInterval);
	if (interval && interval[0] != '\0') {
		const long val = std::strtol(interval, nullptr, 10);
		if (val > 0) {
			summary_interval_ms_ = static_cast< uint32_t >(val);
		}
	}

	out_ = stderr;
	const char *path = std::getenv(kEnvLogfile);
	if (path && path[0] != '\0') {
		FILE *fp = std::fopen(path, "a");
		if (fp) {
			out_ = fp;
		}
	}
}

const char *Trace::errorName_(RxError err) const {
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

void Trace::logf_(const char *fmt, ...) const {
	FILE *out = out_ ? out_ : stderr;
	va_list ap;
	va_start(ap, fmt);
	std::vfprintf(out, fmt, ap);
	va_end(ap);
	std::fflush(out);
}

void Trace::onRxFrame(const Header &hdr) {
	++rx_ok_;
	last_rx_type_ = hdr.type;
	last_rx_seq_ = hdr.seq;
	if (verbose_) {
		logf_("[PROTO] RX_OK type=0x%02X seq=0x%02X len=%u flags=0x%02X\n",
			  hdr.type, hdr.seq, static_cast< unsigned int >(hdr.len),
			  hdr.flags);
	}
}

void Trace::onRxError(RxError err) {
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
	if (verbose_) {
		logf_("[PROTO] RX_ERR err=%s\n", errorName_(err));
	}
}

void Trace::onTx(uint8_t type, uint8_t seq, bool ok) {
	if (ok) {
		++tx_ok_;
	} else {
		++tx_fail_;
	}
	last_tx_type_ = type;
	last_tx_seq_ = seq;
	if (verbose_) {
		logf_("[PROTO] TX_%s type=0x%02X seq=0x%02X\n", ok ? "OK" : "FAIL",
			  type, seq);
	}
}

void Trace::onAck(const AckPayload &payload) {
	if (payload.code == 0) {
		++ack_ok_;
	} else {
		++ack_err_;
		if (verbose_) {
			logf_("[PROTO] ACK code=%u type=0x%02X seq=0x%02X\n", payload.code,
				  payload.type_echo, payload.seq_echo);
		}
	}
}

void Trace::onStatus(const StatusPayload &payload) {
	++status_rx_;
	if (verbose_) {
		logf_("[PROTO] STATUS auto=%u seq=%u fault=0x%04X age=%u\n",
			  payload.auto_active, payload.seq_applied, payload.fault,
			  payload.age_ms);
	}
}

void Trace::tick(uint32_t now_ms) {
	if ((uint32_t)(now_ms - last_summary_ms_) < summary_interval_ms_) {
		return;
	}
	last_summary_ms_ = now_ms;
	const char *err_name = has_err_ ? errorName_(last_err_) : kNone;
	logf_(
		"[PROTO] rx_ok=%lu rx_crc=%lu rx_len=%lu rx_dec=%lu rx_ovf=%lu "
		"rx_ver=%lu rx_unsup=%lu tx_ok=%lu tx_fail=%lu ack_ok=%lu ack_err=%lu "
		"status_rx=%lu last_rx=0x%02X/0x%02X last_tx=0x%02X/0x%02X "
		"last_err=%s\n",
		static_cast< unsigned long >(rx_ok_),
		static_cast< unsigned long >(rx_crc_),
		static_cast< unsigned long >(rx_bad_len_),
		static_cast< unsigned long >(rx_decode_),
		static_cast< unsigned long >(rx_overflow_),
		static_cast< unsigned long >(rx_bad_ver_),
		static_cast< unsigned long >(rx_unsupported_),
		static_cast< unsigned long >(tx_ok_),
		static_cast< unsigned long >(tx_fail_),
		static_cast< unsigned long >(ack_ok_),
		static_cast< unsigned long >(ack_err_),
		static_cast< unsigned long >(status_rx_), last_rx_type_, last_rx_seq_,
		last_tx_type_, last_tx_seq_, err_name);
}

} // namespace proto
