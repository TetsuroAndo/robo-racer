#include "Tsd20.h"
#include "../config/Config.h"

namespace {
static constexpr uint8_t REG_DIST_H = 0x00;
static constexpr uint8_t REG_LASER_CTRL = 0x02;
static constexpr uint8_t REG_ID = 0x03;

static uint8_t calcChecksum(uint8_t cmd, uint8_t len, uint8_t b0, uint8_t b1) {
	const uint16_t sum =
		(uint16_t)cmd + (uint16_t)len + (uint16_t)b0 + (uint16_t)b1;
	return (uint8_t)(0xFFu - (sum & 0xFFu));
}

static bool readAck(HardwareSerial &uart, uint8_t b0, uint8_t b1, uint8_t b2,
					uint32_t timeout_ms) {
	uint8_t buf[16];
	size_t n = 0;
	const uint32_t t0 = millis();
	while (millis() - t0 < timeout_ms) {
		while (uart.available() && n < sizeof(buf)) {
			buf[n++] = (uint8_t)uart.read();
		}
		if (n >= 3)
			break;
	}
	if (n >= 3) {
		for (size_t i = 0; i + 2 < n; i++) {
			if (buf[i] == b0 && buf[i + 1] == b1 && buf[i + 2] == b2) {
				return true;
			}
		}
	}
	return false;
}
} // namespace

bool Tsd20::begin(HardwareSerial &uart) {
	_freq_ack = false;
	_iic_ack = false;

	if (cfg::TSD20_SET_FREQ_ON_BOOT && cfg::TSD20_TARGET_HZ > 0) {
		(void)sendFreqAndChangeIIC(uart, cfg::TSD20_TARGET_HZ);
	}

	uint8_t id = 0;
	if (detectI2C(id)) {
		_ready = true;
		_id = id;
		return true;
	}

	(void)sendChangeIIC(uart);

	delay(50);
	if (detectI2C(id)) {
		_ready = true;
		_id = id;
		return true;
	}

	_ready = false;
	return false;
}

bool Tsd20::readDistanceMm(uint16_t &mm) {
	uint8_t b[2];
	if (!i2cRead(REG_DIST_H, b, 2))
		return false;
	mm = (uint16_t(b[0]) << 8) | b[1];
	return true;
}

bool Tsd20::setLaser(bool on) { return i2cWrite8(REG_LASER_CTRL, on ? 1 : 0); }

bool Tsd20::setFrequencyHz(HardwareSerial &uart, uint16_t hz) {
	if (hz == 0 || hz > 10000)
		return false;
	const uint16_t div = (uint16_t)((10000u / hz) - 1u);
	bool ok = sendFrequencySetting(uart, div);
	if (ok) {
		uint8_t id = 0;
		(void)detectI2C(id);
	}
	return ok;
}

bool Tsd20::detectI2C(uint8_t &id) {
	_ready = false;
	_swapped = false;

	initI2C(false);
	if (probeI2C(id)) {
		_ready = true;
		_swapped = false;
		return true;
	}

	if (cfg::TSD20_ALLOW_PIN_SWAP) {
		initI2C(true);
		if (probeI2C(id)) {
			_ready = true;
			_swapped = true;
			return true;
		}
	}
	return false;
}

void Tsd20::initI2C(bool swapped) {
	const int sda = swapped ? cfg::TSD20_SCL_PIN : cfg::TSD20_SDA_PIN;
	const int scl = swapped ? cfg::TSD20_SDA_PIN : cfg::TSD20_SCL_PIN;

	Wire.end();
	delay(10);
	Wire.begin(sda, scl, cfg::TSD20_I2C_HZ);
	delay(30);
}

bool Tsd20::probeI2C(uint8_t &id) { return i2cRead(REG_ID, &id, 1); }

bool Tsd20::i2cRead(uint8_t reg, uint8_t *buf, size_t len) {
	// repeated-start first
	Wire.beginTransmission(cfg::TSD20_I2C_ADDR);
	Wire.write(reg);
	uint8_t rc = Wire.endTransmission(false);
	if (rc == 0) {
		size_t got =
			Wire.requestFrom((int)cfg::TSD20_I2C_ADDR, (int)len, (int)true);
		if (got == len) {
			for (size_t i = 0; i < len; i++)
				buf[i] = (uint8_t)Wire.read();
			return true;
		}
	}

	// fallback STOP
	Wire.beginTransmission(cfg::TSD20_I2C_ADDR);
	Wire.write(reg);
	rc = Wire.endTransmission(true);
	if (rc != 0)
		return false;

	size_t got =
		Wire.requestFrom((int)cfg::TSD20_I2C_ADDR, (int)len, (int)true);
	if (got != len)
		return false;

	for (size_t i = 0; i < len; i++)
		buf[i] = (uint8_t)Wire.read();
	return true;
}

bool Tsd20::i2cWrite8(uint8_t reg, uint8_t v) {
	Wire.beginTransmission(cfg::TSD20_I2C_ADDR);
	Wire.write(reg);
	Wire.write(v);
	return (Wire.endTransmission(true) == 0);
}

bool Tsd20::sendChangeIIC(HardwareSerial &uart) {
	bool saw_response = false;
	(void)sendChangeIICAtBaud(uart, cfg::TSD20_UART_BAUD_PRIMARY,
							  cfg::TSD20_SCL_PIN, cfg::TSD20_SDA_PIN,
							  saw_response);

	if (!saw_response) {
		(void)sendChangeIICAtBaud(uart, cfg::TSD20_UART_BAUD_FALLBACK,
								  cfg::TSD20_SCL_PIN, cfg::TSD20_SDA_PIN,
								  saw_response);
	}

	if (!saw_response && cfg::TSD20_ALLOW_PIN_SWAP) {
		(void)sendChangeIICAtBaud(uart, cfg::TSD20_UART_BAUD_PRIMARY,
								  cfg::TSD20_SDA_PIN, cfg::TSD20_SCL_PIN,
								  saw_response);
	}

	if (!saw_response && cfg::TSD20_ALLOW_PIN_SWAP) {
		(void)sendChangeIICAtBaud(uart, cfg::TSD20_UART_BAUD_FALLBACK,
								  cfg::TSD20_SDA_PIN, cfg::TSD20_SCL_PIN,
								  saw_response);
	}

	_iic_ack = saw_response;
	return saw_response;
}

bool Tsd20::sendChangeIICAtBaud(HardwareSerial &uart, uint32_t baud, int rx_pin,
								int tx_pin, bool &saw_response) {
	saw_response = false;

	// Change IIC: 5A 1F 02 1F 1F A0
	const uint8_t cmd[] = {0x5A, 0x1F, 0x02, 0x1F, 0x1F, 0xA0};

	Wire.end();
	delay(10);

	uart.end();
	delay(20);
	uart.begin(baud, SERIAL_8N1, rx_pin, tx_pin);
	delay(50);

	while (uart.available())
		(void)uart.read();

	uart.write(cmd, sizeof(cmd));
	uart.flush();

	saw_response = readAck(uart, 0x5A, 0x9F, 0x02, 200);
	uart.end();
	delay(30);

	return saw_response;
}

bool Tsd20::sendFreqAndChangeIIC(HardwareSerial &uart, uint16_t hz) {
	if (hz == 0 || hz > 10000)
		return false;
	const uint16_t div = (uint16_t)((10000u / hz) - 1u);
	bool freq_ok = false;
	bool iic_ok = false;
	bool freq_ack = false;
	bool iic_ack = false;
	(void)sendFreqAndChangeIICAtBaud(uart, cfg::TSD20_UART_BAUD_PRIMARY,
									 cfg::TSD20_SCL_PIN, cfg::TSD20_SDA_PIN,
									 div, freq_ack, iic_ack);
	freq_ok |= freq_ack;
	iic_ok |= iic_ack;

	if (!iic_ok) {
		(void)sendFreqAndChangeIICAtBaud(uart, cfg::TSD20_UART_BAUD_FALLBACK,
										 cfg::TSD20_SCL_PIN, cfg::TSD20_SDA_PIN,
										 div, freq_ack, iic_ack);
		freq_ok |= freq_ack;
		iic_ok |= iic_ack;
	}

	if (!iic_ok && cfg::TSD20_ALLOW_PIN_SWAP) {
		(void)sendFreqAndChangeIICAtBaud(uart, cfg::TSD20_UART_BAUD_PRIMARY,
										 cfg::TSD20_SDA_PIN, cfg::TSD20_SCL_PIN,
										 div, freq_ack, iic_ack);
		freq_ok |= freq_ack;
		iic_ok |= iic_ack;
	}

	if (!iic_ok && cfg::TSD20_ALLOW_PIN_SWAP) {
		(void)sendFreqAndChangeIICAtBaud(uart, cfg::TSD20_UART_BAUD_FALLBACK,
										 cfg::TSD20_SDA_PIN, cfg::TSD20_SCL_PIN,
										 div, freq_ack, iic_ack);
		freq_ok |= freq_ack;
		iic_ok |= iic_ack;
	}

	_freq_ack = freq_ok;
	_iic_ack = iic_ok;
	return iic_ok;
}

bool Tsd20::sendFreqAndChangeIICAtBaud(HardwareSerial &uart, uint32_t baud,
									   int rx_pin, int tx_pin, uint16_t div,
									   bool &freq_ack, bool &iic_ack) {
	freq_ack = false;
	iic_ack = false;

	const uint8_t div_h = (uint8_t)((div >> 8) & 0xFF);
	const uint8_t div_l = (uint8_t)(div & 0xFF);
	const uint8_t chk = calcChecksum(0x0B, 0x02, div_h, div_l);
	const uint8_t cmd_freq[] = {0x5A, 0x0B, 0x02, div_h, div_l, chk};
	const uint8_t cmd_iic[] = {0x5A, 0x1F, 0x02, 0x1F, 0x1F, 0xA0};

	Wire.end();
	delay(10);

	uart.end();
	delay(20);
	uart.begin(baud, SERIAL_8N1, rx_pin, tx_pin);
	delay(50);

	while (uart.available())
		(void)uart.read();

	uart.write(cmd_freq, sizeof(cmd_freq));
	uart.flush();
	freq_ack = readAck(uart, 0x5A, 0x8B, 0x02, 200);

	while (uart.available())
		(void)uart.read();

	uart.write(cmd_iic, sizeof(cmd_iic));
	uart.flush();
	iic_ack = readAck(uart, 0x5A, 0x9F, 0x02, 200);

	uart.end();
	delay(30);
	return iic_ack;
}

bool Tsd20::sendFrequencySetting(HardwareSerial &uart, uint16_t div) {
	bool saw_response = false;
	(void)sendFrequencySettingAtBaud(uart, cfg::TSD20_UART_BAUD_PRIMARY,
									 cfg::TSD20_SCL_PIN, cfg::TSD20_SDA_PIN,
									 div, saw_response);

	if (!saw_response) {
		(void)sendFrequencySettingAtBaud(uart, cfg::TSD20_UART_BAUD_FALLBACK,
										 cfg::TSD20_SCL_PIN, cfg::TSD20_SDA_PIN,
										 div, saw_response);
	}

	if (!saw_response && cfg::TSD20_ALLOW_PIN_SWAP) {
		(void)sendFrequencySettingAtBaud(uart, cfg::TSD20_UART_BAUD_PRIMARY,
										 cfg::TSD20_SDA_PIN, cfg::TSD20_SCL_PIN,
										 div, saw_response);
	}

	if (!saw_response && cfg::TSD20_ALLOW_PIN_SWAP) {
		(void)sendFrequencySettingAtBaud(uart, cfg::TSD20_UART_BAUD_FALLBACK,
										 cfg::TSD20_SDA_PIN, cfg::TSD20_SCL_PIN,
										 div, saw_response);
	}

	_freq_ack = saw_response;
	return saw_response;
}

bool Tsd20::sendFrequencySettingAtBaud(HardwareSerial &uart, uint32_t baud,
									   int rx_pin, int tx_pin, uint16_t div,
									   bool &saw_response) {
	saw_response = false;

	const uint8_t div_h = (uint8_t)((div >> 8) & 0xFF);
	const uint8_t div_l = (uint8_t)(div & 0xFF);
	const uint8_t chk = calcChecksum(0x0B, 0x02, div_h, div_l);
	const uint8_t cmd[] = {0x5A, 0x0B, 0x02, div_h, div_l, chk};

	Wire.end();
	delay(10);

	uart.end();
	delay(20);
	uart.begin(baud, SERIAL_8N1, rx_pin, tx_pin);
	delay(50);

	while (uart.available())
		(void)uart.read();

	uart.write(cmd, sizeof(cmd));
	uart.flush();

	saw_response = readAck(uart, 0x5A, 0x8B, 0x02, 200);
	uart.end();
	delay(30);

	return saw_response;
}
