#pragma once

#include <Arduino.h>
#include <Wire.h>

struct ImuSample {
	int16_t ax = 0;
	int16_t ay = 0;
	int16_t az = 0;
	int16_t temp = 0;
	int16_t gx = 0;
	int16_t gy = 0;
	int16_t gz = 0;
};

class Mpu6500 {
public:
	bool begin(TwoWire &wire);
	bool ready() const { return _ready; }
	uint8_t id() const { return _id; }

	bool readSample(ImuSample &out);

private:
	bool readReg(uint8_t reg, uint8_t &v);
	bool readRegs(uint8_t reg, uint8_t *buf, size_t len);
	bool writeReg(uint8_t reg, uint8_t v);

	TwoWire *_wire = nullptr;
	uint8_t _id = 0;
	bool _ready = false;
};
