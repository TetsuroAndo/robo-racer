#include "Sender.h"
#include "uart.h"

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <time.h>
#include <unistd.h>

namespace {
uint32_t now_ms() {
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	const uint64_t ms = static_cast< uint64_t >(ts.tv_sec) * 1000ULL +
						static_cast< uint64_t >(ts.tv_nsec) / 1000000ULL;
	return static_cast< uint32_t >(ms & 0xFFFFFFFFu);
}

int16_t clamp_speed(int speed) {
	if (speed > cfg::SPEED_LIMIT) {
		speed = cfg::SPEED_LIMIT;
	} else if (speed < -cfg::SPEED_LIMIT) {
		speed = -cfg::SPEED_LIMIT;
	}
	return static_cast< int16_t >(speed);
}

int16_t clamp_cdeg(int32_t cdeg) {
	if (cdeg > std::numeric_limits< int16_t >::max()) {
		cdeg = std::numeric_limits< int16_t >::max();
	} else if (cdeg < std::numeric_limits< int16_t >::min()) {
		cdeg = std::numeric_limits< int16_t >::min();
	}
	return static_cast< int16_t >(cdeg);
}

proto::Trace::RxError toTraceError(proto::PacketReader::Error err) {
	switch (err) {
	case proto::PacketReader::Error::BUFFER_OVERFLOW:
		return proto::Trace::RxError::kOverflow;
	case proto::PacketReader::Error::DECODE_ERROR:
		return proto::Trace::RxError::kDecode;
	case proto::PacketReader::Error::TOO_SHORT:
		return proto::Trace::RxError::kTooShort;
	case proto::PacketReader::Error::BAD_LENGTH:
		return proto::Trace::RxError::kBadLength;
	case proto::PacketReader::Error::CRC_MISMATCH:
		return proto::Trace::RxError::kCrcMismatch;
	default:
		return proto::Trace::RxError::kDecode;
	}
}
} // namespace

Sender::Sender(const char *esp_dev, int esp_baud) { _init(esp_dev, esp_baud); }

Sender::~Sender() {
	if (auto_enabled_) {
		sendAutoMode(false);
	}
	close(_espFd);
}

void Sender::send(int speed, int angle) {
	poll();
	const uint32_t now = now_ms();
	trace_.tick(now);
	if (!auto_enabled_) {
		return;
	}

	maybeSendHeartbeat(now);

	proto::AutoSetpointPayload payload{};
	payload.steer_cdeg =
		clamp_cdeg(static_cast< int32_t >(angle) * cfg::STEER_CDEG_SCALE);
	payload.speed_cmd = clamp_speed(speed);
	payload.ttl_ms = cfg::AUTO_TTL_MS;
	payload.distance_mm = 0;

	const uint8_t seq = nextSeq();
	const bool ok = writer_.write(
		_espFd, static_cast< uint8_t >(proto::Type::AUTO_SETPOINT), 0, seq,
		reinterpret_cast< const uint8_t * >(&payload), sizeof(payload));
	trace_.onTx(static_cast< uint8_t >(proto::Type::AUTO_SETPOINT), seq, ok);
	if (!ok) {
		std::cerr << "AUTO_SETPOINT send failed\n";
	}
}

void Sender::sendAutoMode(bool enable) {
	proto::AutoModePayload payload{};
	payload.enable = enable ? 1 : 0;
	payload.reason = 0;

	const uint8_t seq = nextSeq();
	const bool ok = writer_.write(
		_espFd, static_cast< uint8_t >(proto::Type::AUTO_MODE),
		proto::FLAG_ACK_REQ, seq, reinterpret_cast< const uint8_t * >(&payload),
		sizeof(payload));
	trace_.onTx(static_cast< uint8_t >(proto::Type::AUTO_MODE), seq, ok);
	if (!ok) {
		std::cerr << "AUTO_MODE send failed\n";
		return;
	}
	auto_enabled_ = enable;
	if (!enable) {
		last_hb_ms_ = 0;
	}
}

void Sender::sendKill() {
	const uint8_t seq = nextSeq();
	const bool ok =
		writer_.write(_espFd, static_cast< uint8_t >(proto::Type::KILL),
					  proto::FLAG_ACK_REQ, seq, nullptr, 0);
	trace_.onTx(static_cast< uint8_t >(proto::Type::KILL), seq, ok);
	if (!ok) {
		std::cerr << "KILL send failed\n";
	}
}

void Sender::sendClearKill() {
	const uint8_t seq = nextSeq();
	const bool ok =
		writer_.write(_espFd, static_cast< uint8_t >(proto::Type::CLEAR_KILL),
					  proto::FLAG_ACK_REQ, seq, nullptr, 0);
	trace_.onTx(static_cast< uint8_t >(proto::Type::CLEAR_KILL), seq, ok);
	if (!ok) {
		std::cerr << "CLEAR_KILL send failed\n";
	}
}

void Sender::poll() {
	uint8_t buf[cfg::UART_READ_BUF_SIZE];
	while (true) {
		const ssize_t n = read(_espFd, buf, sizeof(buf));
		if (n == 0) {
			break;
		}
		if (n < 0) {
			if (errno != EAGAIN && errno != EWOULDBLOCK) {
				std::cerr << "UART read error: " << std::strerror(errno)
						  << "\n";
			}
			break;
		}
		for (ssize_t i = 0; i < n; ++i) {
			proto::FrameView frame{};
			const auto res = reader_.push(buf[i], frame);
			if (res == proto::PacketReader::Result::ERROR) {
				trace_.onRxError(toTraceError(reader_.lastError()));
				continue;
			}
			if (res == proto::PacketReader::Result::OK) {
				handleFrame(frame);
			}
		}
	}
	trace_.tick(now_ms());
}

void Sender::_init(const char *esp_dev, int esp_baud) {
	_espFd = uart_open_readwrite(esp_dev, esp_baud);
	if (_espFd < 0) {
		std::cerr << "Failed to open UART: " << std::strerror(errno) << "\n";
		exit(1);
	}

	sendAutoMode(true);
}

void Sender::handleFrame(const proto::FrameView &frame) {
	const auto *hdr = reinterpret_cast< const proto::Header * >(frame.data);
	if (hdr->ver != proto::VER) {
		trace_.onRxError(proto::Trace::RxError::kBadVersion);
		return;
	}
	trace_.onRxFrame(*hdr);

	switch (hdr->type) {
	case static_cast< uint8_t >(proto::Type::ACK): {
		if (hdr->len != sizeof(proto::AckPayload)) {
			return;
		}
		proto::AckPayload payload{};
		memcpy(&payload, frame.data + sizeof(proto::Header), sizeof(payload));
		handleAck(payload, hdr->seq);
		break;
	}
	case static_cast< uint8_t >(proto::Type::STATUS): {
		if (hdr->len != sizeof(proto::StatusPayload)) {
			return;
		}
		proto::StatusPayload payload{};
		memcpy(&payload, frame.data + sizeof(proto::Header), sizeof(payload));
		handleStatus(payload);
		break;
	}
	default:
		trace_.onRxError(proto::Trace::RxError::kUnsupported);
		break;
	}
}

void Sender::handleAck(const proto::AckPayload &payload, uint8_t seq) {
	trace_.onAck(payload);
	if (payload.code == 0) {
		return;
	}
	std::cerr << "ACK error: type=0x" << std::hex
			  << static_cast< int >(payload.type_echo) << " seq=0x"
			  << static_cast< int >(payload.seq_echo) << " code=" << std::dec
			  << static_cast< int >(payload.code) << " rx_seq=0x" << std::hex
			  << static_cast< int >(seq) << std::dec << "\n";
}

void Sender::handleStatus(const proto::StatusPayload &payload) {
	last_status_ = payload;
	has_status_ = true;
	trace_.onStatus(payload);

	const uint32_t now = now_ms();
	if ((uint32_t)(now - last_status_log_ms_) < cfg::STATUS_LOG_INTERVAL_MS) {
		return;
	}
	last_status_log_ms_ = now;

	std::cerr << "STATUS auto=" << static_cast< int >(payload.auto_active)
			  << " seq=" << static_cast< int >(payload.seq_applied)
			  << " fault=0x" << std::hex << payload.fault << std::dec
			  << " speed=" << payload.speed_now
			  << " steer_cdeg=" << payload.steer_now_cdeg
			  << " age_ms=" << payload.age_ms << "\n";
}

void Sender::maybeSendHeartbeat(uint32_t now_ms) {
	if (!auto_enabled_) {
		return;
	}
	if (last_hb_ms_ != 0 &&
		(uint32_t)(now_ms - last_hb_ms_) < cfg::HEARTBEAT_INTERVAL_MS) {
		return;
	}

	const uint8_t seq = nextSeq();
	const bool ok =
		writer_.write(_espFd, static_cast< uint8_t >(proto::Type::HEARTBEAT), 0,
					  seq, nullptr, 0);
	if (ok) {
		last_hb_ms_ = now_ms;
	}
	trace_.onTx(static_cast< uint8_t >(proto::Type::HEARTBEAT), seq, ok);
}

uint8_t Sender::nextSeq() {
	const uint8_t s = seq_;
	seq_ = static_cast< uint8_t >(seq_ + 1);
	return s;
}
