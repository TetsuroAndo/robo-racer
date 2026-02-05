#pragma once

#include <Arduino.h>
#include <Wire.h>

class Tsd20 {
public:
	bool begin(HardwareSerial& uart);

	bool ready() const { return _ready; }
	bool swapped() const { return _swapped; }
	uint8_t id() const { return _id; }

	bool readDistanceMm(uint16_t& mm);
	bool setLaser(bool on);
	bool setFrequencyHz(HardwareSerial& uart, uint16_t hz);

private:
	bool detectI2C(uint8_t& id);
	void initI2C(bool swapped);

	bool probeI2C(uint8_t& id);
	bool i2cRead(uint8_t reg, uint8_t* buf, size_t len);
	bool i2cWrite8(uint8_t reg, uint8_t v);

	bool sendChangeIIC(HardwareSerial& uart);
	bool sendChangeIICAtBaud(HardwareSerial& uart,
		uint32_t baud,
		int rx_pin,
		int tx_pin,
		bool& saw_response);
	bool sendFrequencySetting(HardwareSerial& uart, uint16_t div);
	bool sendFrequencySettingAtBaud(HardwareSerial& uart,
		uint32_t baud,
		int rx_pin,
		int tx_pin,
		uint16_t div,
		bool& saw_response);

	bool _ready = false;
	bool _swapped = false;
	uint8_t _id = 0;
};
