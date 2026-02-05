#include "Mpu6500.h"
#include "../config/Config.h"

namespace {
static constexpr uint8_t REG_SMPLRT_DIV = 0x19;
static constexpr uint8_t REG_CONFIG = 0x1A;
static constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
static constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
static constexpr uint8_t REG_ACCEL_CONFIG2 = 0x1D;
static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
static constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
static constexpr uint8_t REG_WHO_AM_I = 0x75;
} // namespace

bool Mpu6500::begin(TwoWire &wire) {
	_wire = &wire;
	_wire->begin(cfg::IMU_SDA_PIN, cfg::IMU_SCL_PIN, cfg::IMU_I2C_HZ);
	delay(10);

	uint8_t id = 0;
	if (!readReg(REG_WHO_AM_I, id)) {
		_ready = false;
		return false;
	}
	_id = id;
	if (id != cfg::IMU_WHO_AM_I_0 && id != cfg::IMU_WHO_AM_I_1) {
		_ready = false;
		return false;
	}

	if (!writeReg(REG_PWR_MGMT_1, 0x01)) {
		_ready = false;
		return false;
	}
	delay(10);

	if (!writeReg(REG_SMPLRT_DIV, 0x00)) {
		_ready = false;
		return false;
	}
	if (!writeReg(REG_CONFIG, (uint8_t)(cfg::IMU_DLPF_CFG & 0x07))) {
		_ready = false;
		return false;
	}
	if (!writeReg(REG_GYRO_CONFIG,
				  (uint8_t)((cfg::IMU_GYRO_FS_SEL & 0x03) << 3))) {
		_ready = false;
		return false;
	}
	if (!writeReg(REG_ACCEL_CONFIG,
				  (uint8_t)((cfg::IMU_ACCEL_FS_SEL & 0x03) << 3))) {
		_ready = false;
		return false;
	}
	if (!writeReg(REG_ACCEL_CONFIG2, (uint8_t)(cfg::IMU_DLPF_CFG & 0x07))) {
		_ready = false;
		return false;
	}

	_ready = true;
	return true;
}

bool Mpu6500::readSample(ImuSample &out) {
	if (!_ready || !_wire)
		return false;
	uint8_t buf[14];
	if (!readRegs(REG_ACCEL_XOUT_H, buf, sizeof(buf)))
		return false;

	out.ax = (int16_t)((uint16_t(buf[0]) << 8) | buf[1]);
	out.ay = (int16_t)((uint16_t(buf[2]) << 8) | buf[3]);
	out.az = (int16_t)((uint16_t(buf[4]) << 8) | buf[5]);
	out.temp = (int16_t)((uint16_t(buf[6]) << 8) | buf[7]);
	out.gx = (int16_t)((uint16_t(buf[8]) << 8) | buf[9]);
	out.gy = (int16_t)((uint16_t(buf[10]) << 8) | buf[11]);
	out.gz = (int16_t)((uint16_t(buf[12]) << 8) | buf[13]);
	return true;
}

bool Mpu6500::readReg(uint8_t reg, uint8_t &v) { return readRegs(reg, &v, 1); }

bool Mpu6500::readRegs(uint8_t reg, uint8_t *buf, size_t len) {
	if (!_wire)
		return false;
	_wire->beginTransmission(cfg::IMU_I2C_ADDR);
	_wire->write(reg);
	if (_wire->endTransmission(false) != 0)
		return false;

	const size_t got =
		_wire->requestFrom((int)cfg::IMU_I2C_ADDR, (int)len, (int)true);
	if (got != len)
		return false;

	for (size_t i = 0; i < len; ++i)
		buf[i] = (uint8_t)_wire->read();
	return true;
}

bool Mpu6500::writeReg(uint8_t reg, uint8_t v) {
	if (!_wire)
		return false;
	_wire->beginTransmission(cfg::IMU_I2C_ADDR);
	_wire->write(reg);
	_wire->write(v);
	return (_wire->endTransmission(true) == 0);
}
