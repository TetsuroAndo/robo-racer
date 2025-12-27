#include "hardware/Tsd20.h"

namespace mc {

Tsd20::Tsd20(uint8_t addr7bit) : _addr(addr7bit) {}

Result Tsd20::begin(TwoWire &wire) {
	_wire = &wire;
	return Result::Ok();
}

Result Tsd20::setLaserEnabled(bool on) {
	if (!_wire)
		return Result::Fail(Errc::NotReady, "wire not set");
	_wire->beginTransmission(_addr);
	_wire->write(REG_LASER);
	_wire->write(on ? 0x01 : 0x00);
	const uint8_t rc = _wire->endTransmission();
	if (rc != 0)
		return Result::Fail(Errc::Bus, "i2c tx failed");
	return Result::Ok();
}

Result Tsd20::readDistanceMm(uint16_t &outMm) {
	if (!_wire)
		return Result::Fail(Errc::NotReady, "wire not set");

	// Set register pointer
	_wire->beginTransmission(_addr);
	_wire->write(REG_DIST_H);
	uint8_t rc = _wire->endTransmission(false);
	if (rc != 0)
		return Result::Fail(Errc::Bus, "i2c addr/reg failed");

	const uint8_t need = 2;
	const uint8_t got = _wire->requestFrom((int)_addr, (int)need);
	if (got != need)
		return Result::Fail(Errc::Timeout, "i2c read timeout");

	const uint8_t hi = _wire->read();
	const uint8_t lo = _wire->read();
	outMm = ((uint16_t)hi << 8) | lo;

	if (outMm == 0 || outMm > 20000) {
		// treat out-of-range as invalid (sensor supports up to ~20m depending
		// on target)
		return Result::Fail(Errc::Range, "distance out of range");
	}

	return Result::Ok();
}

} // namespace mc
