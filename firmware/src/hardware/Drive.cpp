#include "Drive.h"
#include "../config/Config.h"

namespace {
inline bool isFront(int deg) {
	return (deg <= cfg::DRIVE_FRONT_AREA_DEG) ||
		   (deg >= cfg::FULL_CIRCLE_DEG - cfg::DRIVE_FRONT_AREA_DEG);
}
inline bool isCurveArea(int deg) {
	return (deg <= cfg::DRIVE_CURVE_AREA_DEG) ||
		   (deg >= cfg::FULL_CIRCLE_DEG - cfg::DRIVE_CURVE_AREA_DEG);
}
inline float lerp(float start, float end, float t) {
	return (1 - t) * start + t * end;
}

inline int clampi(int x, int lo, int hi) {
	if (x < lo)
		return lo;
	if (x > hi)
		return hi;
	return x;
}
} // namespace
Drive::Drive() : _angle(0), _speed(0), _lastUpdate(0) {}

Drive::~Drive() {}

void Drive::begin() {
	_engine.begin();
	_steer.begin();

	updateTimeout();
}

void Drive::control() {
	if (evalTimeout() == false) {
		_speed = 0;
		_angle = 0;
	}
	_engine.setSpeed(_speed);
	_steer.setAngle(_angle);
}

void Drive::setSpeed(int newSpeed) {
	updateTimeout();
	_speed = newSpeed;
}

void Drive::setAngle(float newAngle) {
	updateTimeout();
	_angle = newAngle;
}

void Drive::updateTimeout() { _lastUpdate = millis(); }

bool Drive::evalTimeout() {
	return millis() - _lastUpdate < cfg::DRIVE_TIMEOUT_MS;
}

String Drive::info() {
	String res;
	res += String(_speed);
	res += ", ";
	res += String(_angle);
	return res;
}
