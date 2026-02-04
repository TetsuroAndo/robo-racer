#include "Sender.h"
#include "Telemetry.h"
#include "mc/core/Log.hpp"

#include <array>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <sys/socket.h>
#include <time.h>

namespace {
uint32_t now_ms() {
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	const uint64_t ms = static_cast< uint64_t >(ts.tv_sec) * 1000ULL +
						static_cast< uint64_t >(ts.tv_nsec) / 1000000ULL;
	return static_cast< uint32_t >(ms & 0xFFFFFFFFu);
}

int16_t clamp_speed_input(int speed) {
	if (speed > cfg::SPEED_INPUT_LIMIT) {
		speed = cfg::SPEED_INPUT_LIMIT;
	} else if (speed < -cfg::SPEED_INPUT_LIMIT) {
		speed = -cfg::SPEED_INPUT_LIMIT;
	}
	return static_cast< int16_t >(speed);
}

int16_t clamp_cdeg(int32_t cdeg) {
	if (cdeg > cfg::STEER_CDEG_MAX) {
		cdeg = cfg::STEER_CDEG_MAX;
	} else if (cdeg < -cfg::STEER_CDEG_MAX) {
		cdeg = -cfg::STEER_CDEG_MAX;
	}
	return static_cast< int16_t >(cdeg);
}

} // namespace

Sender::Sender(const char *sock_path, TelemetryEmitter *telemetry)
	: telemetry_(telemetry) {
	_init(sock_path);
}

Sender::~Sender() {
	if (auto_enabled_) {
		sendAutoMode(false);
	}
}

void Sender::send(int speed, int angle) {
	poll();
	sendHeartbeatIfDue();
	if (!auto_enabled_) {
		return;
	}

	mc::proto::DrivePayload payload{};
	payload.steer_cdeg =
		clamp_cdeg(static_cast< int32_t >(angle) * cfg::STEER_CDEG_SCALE);
	const int16_t speed_input = clamp_speed_input(speed);
	const int32_t speed_mm_s =
		(int32_t)speed_input * cfg::SPEED_MM_S_MAX / cfg::SPEED_INPUT_LIMIT;
	payload.speed_mm_s = static_cast< int16_t >(speed_mm_s);
	payload.ttl_ms_le = mc::proto::to_le16(cfg::AUTO_TTL_MS);
	payload.dist_mm_le = mc::proto::to_le16(0);

	uint8_t out[mc::proto::MAX_FRAME_ENCODED];
	size_t out_len = 0;
	const uint16_t seq = nextSeq();
	const bool ok = mc::proto::PacketWriter::build(
		out, sizeof(out), out_len, mc::proto::Type::DRIVE, 0, seq,
		reinterpret_cast< const uint8_t * >(&payload), sizeof(payload));
	if (!ok || ::send(ipc_.fd(), out, out_len, MSG_NOSIGNAL) <= 0) {
		MC_LOGW("sender", "DRIVE send failed");
	}
}

void Sender::sendHeartbeatIfDue() {
	const uint32_t now = now_ms();
	if ((uint32_t)(now - last_hb_ms_) < cfg::HEARTBEAT_INTERVAL_MS) {
		return;
	}
	last_hb_ms_ = now;

	const uint16_t seq = nextSeq();
	uint8_t out[mc::proto::MAX_FRAME_ENCODED];
	size_t out_len = 0;
	const bool ok = mc::proto::PacketWriter::build(
		out, sizeof(out), out_len, mc::proto::Type::PING, 0, seq, nullptr, 0);
	if (!ok || ::send(ipc_.fd(), out, out_len, MSG_NOSIGNAL) <= 0) {
		MC_LOGW("sender", "PING send failed");
	}
}

void Sender::sendAutoMode(bool enable) {
	uint8_t mode = enable ? 1 : 0;
	const uint16_t seq = nextSeq();
	uint8_t out[mc::proto::MAX_FRAME_ENCODED];
	size_t out_len = 0;
	const bool ok = mc::proto::PacketWriter::build(
		out, sizeof(out), out_len, mc::proto::Type::MODE_SET,
		mc::proto::FLAG_ACK_REQ, seq, &mode, 1);
	if (!ok || ::send(ipc_.fd(), out, out_len, MSG_NOSIGNAL) <= 0) {
		MC_LOGW("sender", "AUTO_MODE send failed");
		return;
	}
	trackPending(seq, out, (uint16_t)out_len);
	auto_enabled_ = enable;
}

void Sender::sendKill() {
	uint8_t payload[2] = {0, 0};
	const uint16_t seq = nextSeq();
	uint8_t out[mc::proto::MAX_FRAME_ENCODED];
	size_t out_len = 0;
	const bool ok = mc::proto::PacketWriter::build(
		out, sizeof(out), out_len, mc::proto::Type::KILL,
		mc::proto::FLAG_ACK_REQ, seq, payload, sizeof(payload));
	if (!ok || ::send(ipc_.fd(), out, out_len, MSG_NOSIGNAL) <= 0) {
		MC_LOGW("sender", "KILL send failed");
		return;
	}
	trackPending(seq, out, (uint16_t)out_len);
}

void Sender::poll() {
	checkPending();
	checkStatusLiveness();
	uint8_t buf[mc::proto::MAX_FRAME_ENCODED];
	while (true) {
		int n = (int)::recv(ipc_.fd(), buf, (int)sizeof(buf), 0);
		if (n <= 0)
			break;
		mc::proto::Frame frame{};
		std::array< uint8_t, mc::proto::MAX_FRAME_DECODED > decoded{};
		if (mc::proto::decode_one(buf, (size_t)n, frame, decoded)) {
			handleFrame(frame);
		}
	}
	checkPending();
	checkStatusLiveness();
}

void Sender::_init(const char *sock_path) {
	if (!ipc_.connect(sock_path)) {
		MC_LOGE("sender",
				std::string("Failed to connect seriald socket: ") + sock_path);
		exit(1);
	}

	sendAutoMode(true);
}

void Sender::handleFrame(const mc::proto::Frame &frame) {
	if (frame.type() == (uint8_t)mc::proto::Type::STATUS &&
		frame.payload_len == sizeof(mc::proto::StatusPayload)) {
		mc::proto::StatusPayload payload{};
		memcpy(&payload, frame.payload, sizeof(payload));
		handleStatus(payload);
	} else if (frame.type() == (uint8_t)mc::proto::Type::ACK &&
			   frame.payload_len == 0) {
		const uint16_t seq = frame.seq();
		handleAck(seq);
	}
}

void Sender::handleStatus(const mc::proto::StatusPayload &payload) {
	last_status_ = payload;
	has_status_ = true;
	last_status_ms_ = now_ms();
	status_stale_ = false;

	const uint32_t now = now_ms();
	if ((uint32_t)(now - last_status_log_ms_) < cfg::STATUS_LOG_INTERVAL_MS) {
		return;
	}
	last_status_log_ms_ = now;

	const uint8_t seq = payload.seq_applied;
	const uint8_t auto_active = payload.auto_active;
	const uint16_t faults = mc::proto::from_le16(payload.faults_le);
	const int16_t speed =
		(int16_t)mc::proto::from_le16((uint16_t)payload.speed_mm_s_le);
	const int16_t steer =
		(int16_t)mc::proto::from_le16((uint16_t)payload.steer_cdeg_le);
	const uint16_t age_ms = mc::proto::from_le16(payload.age_ms_le);
	std::ostringstream ss;
	ss << "STATUS seq=" << (unsigned)seq << " auto=" << (unsigned)auto_active
	   << " speed_mm_s=" << speed << " steer_cdeg=" << steer
	   << " age_ms=" << age_ms << " faults=0x" << std::hex << faults
	   << std::dec;
	MC_LOGI("status", ss.str());

	if (telemetry_) {
		telemetry_->updateStatus(auto_active, faults, speed, steer, age_ms);
	}
}

void Sender::handleAck(uint16_t seq) {
	auto it = pending_.find(seq);
	if (it != pending_.end()) {
		pending_.erase(it);
	}
}

void Sender::trackPending(uint16_t seq, const uint8_t *data, uint16_t len) {
	if (len == 0 || len > mc::proto::MAX_FRAME_ENCODED)
		return;
	PendingTx p{};
	p.deadline_ms = now_ms() + cfg::ACK_TIMEOUT_MS;
	p.retries = 0;
	p.len = len;
	memcpy(p.data.data(), data, len);
	pending_[seq] = p;
}

void Sender::checkPending() {
	if (pending_.empty())
		return;
	const uint32_t now = now_ms();
	for (auto it = pending_.begin(); it != pending_.end();) {
		PendingTx &p = it->second;
		if ((int32_t)(now - p.deadline_ms) < 0) {
			++it;
			continue;
		}
		if (p.retries >= cfg::ACK_MAX_RETRY) {
			MC_LOGW("sender", "ACK timeout seq=" + std::to_string(it->first));
			it = pending_.erase(it);
			continue;
		}
		if (::send(ipc_.fd(), p.data.data(), p.len, MSG_NOSIGNAL) <= 0) {
			MC_LOGW("sender",
					"ACK retry send failed seq=" + std::to_string(it->first));
		}
		p.retries = (uint8_t)(p.retries + 1);
		p.deadline_ms = now + cfg::ACK_TIMEOUT_MS;
		++it;
	}
}

void Sender::checkStatusLiveness() {
	if (!has_status_)
		return;
	const uint32_t now = now_ms();
	if ((uint32_t)(now - last_status_ms_) > cfg::STATUS_DEAD_MS) {
		if (!status_stale_) {
			status_stale_ = true;
			MC_LOGW("status", "STATUS stale (> " +
								  std::to_string(cfg::STATUS_DEAD_MS) + " ms)");
		}
	}
}

uint16_t Sender::nextSeq() {
	const uint16_t s = seq_;
	seq_ = (uint16_t)(seq_ + 1);
	return s;
}
