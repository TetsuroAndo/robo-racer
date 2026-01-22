#pragma once

#include "Protocol.h"
#include <cstdio>
#include <stdint.h>

namespace proto {

class Trace {
public:
	enum class RxError : uint8_t {
		kOverflow,
		kDecode,
		kTooShort,
		kBadLength,
		kCrcMismatch,
		kBadVersion,
		kUnsupported,
	};

	Trace();
	~Trace();

	void onRxFrame(const Header &hdr);
	void onRxError(RxError err);
	void onTx(uint8_t type, uint8_t seq, bool ok);
	void onAck(const AckPayload &payload);
	void onStatus(const StatusPayload &payload);
	void tick(uint32_t now_ms);

private:
	void configureFromEnv_();
	void logf_(const char *fmt, ...) const;
	const char *errorName_(RxError err) const;

	uint32_t rx_ok_ = 0;
	uint32_t rx_overflow_ = 0;
	uint32_t rx_decode_ = 0;
	uint32_t rx_too_short_ = 0;
	uint32_t rx_bad_len_ = 0;
	uint32_t rx_crc_ = 0;
	uint32_t rx_bad_ver_ = 0;
	uint32_t rx_unsupported_ = 0;

	uint32_t tx_ok_ = 0;
	uint32_t tx_fail_ = 0;
	uint32_t ack_ok_ = 0;
	uint32_t ack_err_ = 0;
	uint32_t status_rx_ = 0;

	uint8_t last_rx_type_ = 0;
	uint8_t last_rx_seq_ = 0;
	uint8_t last_tx_type_ = 0;
	uint8_t last_tx_seq_ = 0;
	bool has_err_ = false;
	RxError last_err_ = RxError::kOverflow;

	bool verbose_ = false;
	uint32_t summary_interval_ms_ = 1000;
	uint32_t last_summary_ms_ = 0;

	FILE *out_ = nullptr;
};

} // namespace proto
