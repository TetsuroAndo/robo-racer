#pragma once
#include <WiFi.h>
#include <WiFiUdp.h>
#include "comm/protocol/PacketWriter.h"
#include "log/Log.h"

namespace mc::log {

class ProtocolSink : public Sink {
 public:
	explicit ProtocolSink(Stream &out);
	void write(const Record &rec, const char *message) override;

 private:
	Stream &_out;
	proto::PacketEncoder _encoder;
};

class UdpSink : public Sink {
 public:
	UdpSink();
	bool begin(const char *ssid, const char *pass, const char *host,
			   uint16_t port, uint32_t timeout_ms);
	bool ready() const { return _ready; }
	void write(const Record &rec, const char *message) override;

 private:
	bool connectWifi(const char *ssid, const char *pass, uint32_t timeout_ms);
	bool resolveHost(const char *host);

	proto::PacketEncoder _encoder;
	WiFiUDP _udp;
	IPAddress _remote;
	bool _ready = false;
	uint16_t _port = 0;
};

} // namespace mc::log
