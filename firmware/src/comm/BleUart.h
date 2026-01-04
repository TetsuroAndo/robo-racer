#pragma once
#include <Arduino.h>
#include <Stream.h>
#include <NimBLEDevice.h>

namespace mc {

class BleUart : public Stream {
 public:
	BleUart();

	void begin(const char *deviceName, uint16_t mtu);
	void poll();
	bool connected() const { return _connected; }

	size_t write(uint8_t b) override;
	size_t write(const uint8_t *buffer, size_t size) override;
	int available() override;
	int read() override;
	int peek() override;
	void flush() override;

	using Print::write;

 private:
	class ServerCallbacks : public NimBLEServerCallbacks {
	 public:
		explicit ServerCallbacks(BleUart *owner) : _owner(owner) {}
		void onConnect(NimBLEServer *server) override;
		void onDisconnect(NimBLEServer *server) override;

	 private:
		BleUart *_owner = nullptr;
	};

	class RxCallbacks : public NimBLECharacteristicCallbacks {
	 public:
		explicit RxCallbacks(BleUart *owner) : _owner(owner) {}
		void onWrite(NimBLECharacteristic *ch) override;

	 private:
		BleUart *_owner = nullptr;
	};

	friend class ServerCallbacks;
	friend class RxCallbacks;

	static constexpr size_t kRxBufSize = 512;
	static constexpr size_t kTxChunk = 20;

	void setConnected(bool connected);
	void handleWrite(const uint8_t *data, size_t len);
	bool pushByte(uint8_t b);
	int popByte();
	int peekByte();
	size_t availableBytes();

	NimBLEServer *_server = nullptr;
	NimBLEAdvertising *_advertising = nullptr;
	NimBLECharacteristic *_txChar = nullptr;
	NimBLECharacteristic *_rxChar = nullptr;
	volatile bool _connected = false;

	uint8_t _rxBuf[kRxBufSize];
	volatile size_t _rxHead = 0;
	volatile size_t _rxTail = 0;
	portMUX_TYPE _rxMux = portMUX_INITIALIZER_UNLOCKED;

	ServerCallbacks _serverCallbacks;
	RxCallbacks _rxCallbacks;
};

} // namespace mc
