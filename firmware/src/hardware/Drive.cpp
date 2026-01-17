#include "Drive.h"

namespace {
bool isFront(float deg) { return 0 + 45 <= deg || 360 - 45 <= deg; }
} // namespace

Drive::Drive() : _angle(0), _speed(0) {}

Drive::~Drive() {}

void Drive::begin() {
	_engine.begin();
	_steer.begin();
}

void Drive::control() {
	_engine.setSpeed(_speed);
	_steer.setAngle(_angle);
}

void Drive::evalInput(int lidarDeg, int distance) {
	if (isFront(lidarDeg)) {
		const int min_val = 500;
		int tmp_dist = std::min(min_val, distance);
		float tmp_speed = distance / min_val;
		_speed = (_speed + tmp_speed * 255) / 2;
	}
}
