#include "comm/BleUart.h"
#include <string>

namespace mc {

static const char *kNusServiceUuid = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char *kNusRxUuid = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
static const char *kNusTxUuid = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

BleUart::BleUart() : _serverCallbacks(this), _rxCallbacks(this) {}

void BleUart::begin(const char *deviceName, uint16_t mtu) {
	NimBLEDevice::init(deviceName);
	if (mtu > 0)
		NimBLEDevice::setMTU(mtu);

	_server = NimBLEDevice::createServer();
	_server->setCallbacks(&_serverCallbacks);

	NimBLEService *service = _server->createService(kNusServiceUuid);
	_txChar =
		service->createCharacteristic(kNusTxUuid, NIMBLE_PROPERTY::NOTIFY);

	_rxChar = service->createCharacteristic(
		kNusRxUuid, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
	_rxChar->setCallbacks(&_rxCallbacks);

	service->start();

	_advertising = NimBLEDevice::getAdvertising();
	_advertising->addServiceUUID(service->getUUID());
	_advertising->setScanResponse(true);
	_advertising->start();
}

void BleUart::poll() {}

size_t BleUart::write(uint8_t b) { return write(&b, 1); }

size_t BleUart::write(const uint8_t *buffer, size_t size) {
	if (!_connected || !_txChar || !buffer || size == 0)
		return 0;

	size_t sent = 0;
	while (sent < size) {
		const size_t remaining = size - sent;
		const size_t chunk = remaining > kTxChunk ? kTxChunk : remaining;
		_txChar->setValue(buffer + sent, chunk);
		_txChar->notify();
		sent += chunk;
	}
	return sent;
}

int BleUart::available() { return (int)availableBytes(); }

int BleUart::read() { return popByte(); }

int BleUart::peek() { return peekByte(); }

void BleUart::flush() {}

void BleUart::setConnected(bool connected) { _connected = connected; }

void BleUart::handleWrite(const uint8_t *data, size_t len) {
	for (size_t i = 0; i < len; ++i)
		pushByte(data[i]);
}

bool BleUart::pushByte(uint8_t b) {
	portENTER_CRITICAL(&_rxMux);
	const size_t next = (_rxHead + 1) % kRxBufSize;
	if (next == _rxTail) {
		portEXIT_CRITICAL(&_rxMux);
		return false;
	}
	_rxBuf[_rxHead] = b;
	_rxHead = next;
	portEXIT_CRITICAL(&_rxMux);
	return true;
}

int BleUart::popByte() {
	portENTER_CRITICAL(&_rxMux);
	if (_rxHead == _rxTail) {
		portEXIT_CRITICAL(&_rxMux);
		return -1;
	}
	const uint8_t b = _rxBuf[_rxTail];
	_rxTail = (_rxTail + 1) % kRxBufSize;
	portEXIT_CRITICAL(&_rxMux);
	return b;
}

int BleUart::peekByte() {
	portENTER_CRITICAL(&_rxMux);
	if (_rxHead == _rxTail) {
		portEXIT_CRITICAL(&_rxMux);
		return -1;
	}
	const uint8_t b = _rxBuf[_rxTail];
	portEXIT_CRITICAL(&_rxMux);
	return b;
}

size_t BleUart::availableBytes() {
	portENTER_CRITICAL(&_rxMux);
	const size_t head = _rxHead;
	const size_t tail = _rxTail;
	portEXIT_CRITICAL(&_rxMux);
	if (head >= tail)
		return head - tail;
	return kRxBufSize - (tail - head);
}

void BleUart::ServerCallbacks::onConnect(NimBLEServer *server) {
	(void)server;
	_owner->setConnected(true);
}

void BleUart::ServerCallbacks::onDisconnect(NimBLEServer *server) {
	(void)server;
	_owner->setConnected(false);
	if (_owner->_advertising)
		_owner->_advertising->start();
}

void BleUart::RxCallbacks::onWrite(NimBLECharacteristic *ch) {
	const std::string value = ch->getValue();
	if (!value.empty()) {
		_owner->handleWrite(reinterpret_cast< const uint8_t * >(value.data()),
							value.size());
	}
}

} // namespace mc
