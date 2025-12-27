#include "hardware/Imu.h"
#include <common/Math.h>

// External dep
#include <MPU6050_light.h>

namespace mc {

static bool isStationary(float gyroZ_deg_s) {
	// A conservative stationary detector for bias learning
	return fabsf(gyroZ_deg_s) < 1.0f;
}

Result Imu::begin(TwoWire &wire) {
	static MPU6050 mpu(wire);
	_mpu = (void *)&mpu;

	wire.begin();
	// mpu.begin() returns byte status in this lib
	const uint8_t status = mpu.begin();
	if (status != 0) {
		_healthy = false;
		return Result::Fail(Errc::Bus, "MPU6050 begin failed");
	}

	// Calibrate offsets (keep car still)
	mpu.calcOffsets(true, true);
	_healthy = true;

	_lastMs = millis();
	_yawDeg = 0.0f;
	_yawRateDegS = 0.0f;
	_biasZ = 0.0f;
	_biasSamples = 0;
	_biasLocked = false;

	return Result::Ok();
}

void Imu::updateBias(float gyroZ, float dt_s, bool stationary) {
	(void)dt_s;
	if (_biasLocked)
		return;

	if (stationary) {
		// Running average during first seconds of stillness.
		_biasSamples++;
		const float alpha = 1.0f / (float)_biasSamples;
		_biasZ = (1.0f - alpha) * _biasZ + alpha * gyroZ;

		if (_biasSamples >= 400) { // ~4s at 100Hz
			_biasLocked = true;
		}
	} else {
		// reset accumulation if we start moving
		_biasSamples = 0;
		_biasZ = 0.0f;
	}
}

bool Imu::update() {
	if (!_healthy)
		return false;

	auto *mpu = reinterpret_cast< MPU6050 * >(_mpu);
	mpu->update();

	const uint32_t now = millis();
	const float dt_s = (now > _lastMs) ? ((now - _lastMs) / 1000.0f) : 0.0f;
	_lastMs = now;
	if (dt_s <= 0.0f || dt_s > 0.2f)
		return false;

	const float gyroZ = mpu->getGyroZ(); // deg/s
	updateBias(gyroZ, dt_s, isStationary(gyroZ));

	const float gyroZ_unbiased = gyroZ - _biasZ;
	_yawRateDegS = gyroZ_unbiased;
	_yawDeg = mc::wrapDeg180(_yawDeg + gyroZ_unbiased * dt_s);

	return true;
}

} // namespace mc
