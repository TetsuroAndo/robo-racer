#include "log/LogSinks.h"
#include "comm/protocol/Codec.h"
#include <string.h>

namespace mc::log {

static constexpr size_t kMaxLogMessage = 96;
static constexpr size_t kLogHeaderSize = sizeof(proto::LogPayloadHeader);
static constexpr size_t kMaxPayload = kLogHeaderSize + kMaxLogMessage;
static constexpr size_t kMaxPacket =
	proto::PacketEncoder::kHeaderSize + kMaxPayload;
static constexpr size_t kMaxEncoded = kMaxPacket + (kMaxPacket / 254) + 2;

ProtocolSink::ProtocolSink(Stream &out) : _out(out) {}

void ProtocolSink::write(const Record &rec, const char *message) {
	uint8_t payload[kMaxPayload];
	const size_t msg_len = strnlen(message, kMaxLogMessage);

	proto::LogPayloadHeader header{};
	header.now_ms = rec.now_ms;
	header.level = static_cast< uint8_t >(rec.level);
	header.topic = static_cast< uint8_t >(rec.topic);
	header.msg_len = (uint16_t)msg_len;

	memcpy(payload, &header, sizeof(header));
	if (msg_len > 0)
		memcpy(payload + sizeof(header), message, msg_len);

	uint8_t packet[kMaxPacket];
	size_t packet_len = 0;
	if (!_encoder.build(proto::MsgType::Log, payload,
						(uint16_t)(sizeof(header) + msg_len), 0, packet,
						sizeof(packet), &packet_len)) {
		return;
	}

	uint8_t encoded[kMaxEncoded];
	const size_t encoded_len =
		proto::cobsEncode(packet, packet_len, encoded, sizeof(encoded));
	if (encoded_len == 0)
		return;

	_out.write(encoded, encoded_len);
	_out.write((uint8_t)0x00);
}

UdpSink::UdpSink() {}

bool UdpSink::begin(const char *ssid, const char *pass, const char *host,
					uint16_t port, uint32_t timeout_ms) {
	_ready = false;
	_port = port;
	if (!host || !*host || port == 0)
		return false;
	if (!resolveHost(host))
		return false;
	if (!connectWifi(ssid, pass, timeout_ms))
		return false;
	_ready = true;
	return true;
}

bool UdpSink::connectWifi(const char *ssid, const char *pass,
						  uint32_t timeout_ms) {
	if (WiFi.status() == WL_CONNECTED)
		return true;
	if (!ssid || !*ssid)
		return false;
	WiFi.mode(WIFI_STA);
	WiFi.begin(ssid, pass);
	const uint32_t start = millis();
	while (WiFi.status() != WL_CONNECTED) {
		if ((millis() - start) >= timeout_ms)
			break;
		delay(100);
	}
	return WiFi.status() == WL_CONNECTED;
}

bool UdpSink::resolveHost(const char *host) { return _remote.fromString(host); }

void UdpSink::write(const Record &rec, const char *message) {
	if (!_ready || WiFi.status() != WL_CONNECTED)
		return;

	uint8_t payload[kMaxPayload];
	const size_t msg_len = strnlen(message, kMaxLogMessage);

	proto::LogPayloadHeader header{};
	header.now_ms = rec.now_ms;
	header.level = static_cast< uint8_t >(rec.level);
	header.topic = static_cast< uint8_t >(rec.topic);
	header.msg_len = (uint16_t)msg_len;

	memcpy(payload, &header, sizeof(header));
	if (msg_len > 0)
		memcpy(payload + sizeof(header), message, msg_len);

	uint8_t packet[kMaxPacket];
	size_t packet_len = 0;
	if (!_encoder.build(proto::MsgType::Log, payload,
						(uint16_t)(sizeof(header) + msg_len), 0, packet,
						sizeof(packet), &packet_len)) {
		return;
	}

	uint8_t encoded[kMaxEncoded];
	const size_t encoded_len =
		proto::cobsEncode(packet, packet_len, encoded, sizeof(encoded));
	if (encoded_len == 0)
		return;

	_udp.beginPacket(_remote, _port);
	_udp.write(encoded, encoded_len);
	_udp.write((uint8_t)0x00);
	_udp.endPacket();
}

} // namespace mc::log
