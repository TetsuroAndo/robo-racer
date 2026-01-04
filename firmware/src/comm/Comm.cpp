#include "comm/Comm.h"
#include <common/Math.h>

// External libs (declared via platformio.ini or vendored in
// firmware/lib/external):
// - PacketSerial (COBS framing)
// - AceCRC (CRC16-CCITT)
// We keep includes isolated so users can swap implementations.

#ifndef MC_USE_PACKET_SERIAL
#define MC_USE_PACKET_SERIAL 1
#endif

#if MC_USE_PACKET_SERIAL
#include <PacketSerial.h>
#endif

#include <AceCRC.h>

namespace mc {

#if MC_USE_PACKET_SERIAL
namespace {
struct CommPacketSerial : public PacketSerial_< COBS, 0, 512 > {
	Comm *owner = nullptr;
};
} // namespace
#endif

static uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
	// AceCRC provides multiple CRCs; we use CRC16-CCITT (XModem/CCITT variants
	// exist). For interoperability, *fix the variant*. Here we use
	// crc16ccitt_nibble (poly 0x1021, init 0x1D0F) from AceCRC. If you need a
	// different init/xor, change both ends consistently.
	return ace_crc::crc16ccitt_nibble::crc_calculate(data, len);
}

Comm::Comm() {}

Result Comm::begin(Stream &stream) {
	_io = &stream;
	_lastHostSeenMs = 0;

#if MC_USE_PACKET_SERIAL
	auto *ps = reinterpret_cast< CommPacketSerial * >(_packetSerial);
	if (!ps) {
		ps = new CommPacketSerial();
		_packetSerial = (void *)ps;
	}
	ps->owner = this;
	ps->setStream(_io);
	ps->setPacketHandler(&Comm::handlePacketThunk);
#endif
	return Result::Ok();
}

Result Comm::begin(HardwareSerial &serial, uint32_t baud) {
	serial.begin(baud);
	return begin(static_cast< Stream & >(serial));
}

void Comm::poll() {
#if MC_USE_PACKET_SERIAL
	auto *ps = reinterpret_cast< CommPacketSerial * >(_packetSerial);
	if (!ps)
		return;
	ps->update();
#else
	// Fallback (not framed): do nothing
#endif
}

HostCommand Comm::consumeCommand() {
	HostCommand c = _pending;
	_pending = HostCommand{};
	return c;
}

void Comm::sendStatus(const proto::StatusPayload &st) {
	if (!_io)
		return;

	proto::Header h{};
	h.version = proto::VERSION;
	h.msg_type = (uint8_t)proto::MsgType::Status;
	h.seq = _txSeq++;
	h.flags = 0;
	h.payload_len = (uint16_t)sizeof(st);

	// Build buffer in stack (small)
	uint8_t buf[sizeof(proto::Header) + sizeof(st)];
	memcpy(buf, &h, sizeof(h));
	memcpy(buf + sizeof(h), &st, sizeof(st));

	// Fill CRC over [version..payload], excluding crc field itself.
	// We temporarily zero crc field.
	reinterpret_cast< proto::Header * >(buf)->crc16 = 0;
	const uint16_t crc = crc16_ccitt(buf, sizeof(buf));
	reinterpret_cast< proto::Header * >(buf)->crc16 = crc;

#if MC_USE_PACKET_SERIAL
	auto *ps = reinterpret_cast< CommPacketSerial * >(_packetSerial);
	if (!ps)
		return;
	ps->send(buf, sizeof(buf));
#else
	_io->write(buf, sizeof(buf));
	_io->write('\n');
#endif
}

static bool parseHeader(const uint8_t *buf, size_t len, proto::Header &out) {
	if (len < sizeof(proto::Header))
		return false;
	memcpy(&out, buf, sizeof(proto::Header));
	if (out.version != proto::VERSION)
		return false;
	if (sizeof(proto::Header) + out.payload_len != len)
		return false;
	return true;
}

void Comm::onPacket(const uint8_t *buf, size_t len) {
	proto::Header h{};
	if (!parseHeader(buf, len, h))
		return;

	// CRC check
	uint8_t tmp[1024];
	if (len > sizeof(tmp))
		return;
	memcpy(tmp, buf, len);
	reinterpret_cast< proto::Header * >(tmp)->crc16 = 0;
	const uint16_t crc = crc16_ccitt(tmp, len);
	if (crc != h.crc16)
		return;

	const uint8_t *payload = buf + sizeof(proto::Header);

	_lastHostSeenMs = millis();

	auto mt = (proto::MsgType)h.msg_type;
	if (mt == proto::MsgType::Heartbeat) {
		_pending.heartbeat = true;
		return;
	}
	if (mt == proto::MsgType::Estop) {
		_pending.estop = true;
		return;
	}
	if (mt == proto::MsgType::ClearEstop) {
		_pending.clearEstop = true;
		return;
	}
	if (mt == proto::MsgType::SetPrefer && h.payload_len >= 1) {
		_pending.hasPrefer = true;
		_pending.preferRight = payload[0] != 0;
		return;
	}
	if (mt == proto::MsgType::SetMode && h.payload_len >= 1) {
		_pending.hasMode = true;
		_pending.mode = (proto::Mode)payload[0];
		return;
	}
	if (mt == proto::MsgType::SetManual && h.payload_len >= 4) {
		int16_t thr_m = 0, st_m = 0;
		memcpy(&thr_m, payload + 0, 2);
		memcpy(&st_m, payload + 2, 2);
		_pending.hasManual = true;
		_pending.throttle = mc::clamp((float)thr_m / 1000.0f, -1.0f, 1.0f);
		_pending.steer = mc::clamp((float)st_m / 1000.0f, -1.0f, 1.0f);
		return;
	}
}

void Comm::handlePacketThunk(const void *sender, const uint8_t *buf,
							 size_t len) {
#if MC_USE_PACKET_SERIAL
	const auto *ps = reinterpret_cast< const CommPacketSerial * >(sender);
	if (!ps || !ps->owner)
		return;
	ps->owner->onPacket(buf, len);
#else
	(void)sender;
	(void)buf;
	(void)len;
#endif
}

} // namespace mc
