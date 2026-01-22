#pragma once

#include <Arduino.h>
#include <stdint.h>

class ProtoTrace {
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

	void onRxFrame(uint8_t type, uint8_t seq, uint16_t len, uint8_t flags);
	void onRxError(RxError err);
	void onTx(uint8_t type, uint8_t seq, uint16_t len, uint8_t flags,
			  bool ok);
	void tick(uint32_t now_ms);

private:
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

	uint8_t last_rx_type_ = 0;
	uint8_t last_rx_seq_ = 0;
	uint8_t last_tx_type_ = 0;
	uint8_t last_tx_seq_ = 0;
	bool has_err_ = false;
	RxError last_err_ = RxError::kOverflow;

	uint32_t last_summary_ms_ = 0;

	const char *errorName_(RxError err) const;
	void logSummary_() const;
};
