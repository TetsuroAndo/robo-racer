#include "Drive.h"

#define SPEED_MAX 255
#define AVE_DEG_NUM 1
#define FRONT_AREA 45
#define CURVE_AREA 70

namespace {
inline bool isFront(int deg) {
	return (deg <= FRONT_AREA) || (deg >= 360 - FRONT_AREA);
}
inline bool isCurveArea(int deg) {
	return (deg <= CURVE_AREA) || (deg >= 360 - CURVE_AREA);
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
Drive::Drive() : _angle(0), _speed(0) {}

Drive::~Drive() {}

void Drive::begin() {
	_engine.begin();
	_steer.begin();
}

void Drive::control() {
	int bestDeg = -1;
	int bestDist = -1;

	// 前方±CURVE_AREAの中で最大距離を探す
	for (int i = 0; i < 360; i++) {
		if (!isCurveArea(i))
			continue;
		if (_angles.count(i) == 0)
			continue;

		int d = _angles[i];
		if (d <= 0)
			continue;

		if (d > bestDist) {
			bestDist = d;
			bestDeg = i;
		}
	}

	// bestDeg から「右:+ / 左:-」の正規化量 turn [-1..+1] を作る
	float turn = 0.0f;
	if (bestDeg != -1) {
		if (bestDeg <= CURVE_AREA) {
			// 0..CURVE_AREA は右側（+）
			turn = float(bestDeg) / float(CURVE_AREA); // 0..1
		} else {
			// 360-CURVE_AREA..359 は左側（-）
			turn = -float(360 - bestDeg) / float(CURVE_AREA); // 0..-1
		}
	}

	_angle = 0.5f + 0.5f * turn;

	if (_angle < 0.0f)
		_angle = 0.0f;
	if (_angle > 1.0f)
		_angle = 1.0f;

	_engine.setSpeed(_speed);
	_steer.setAngle(_angle);
	_angles.clear();
}

void Drive::evalInput(int lidarDeg, int distance) {
	_angles[lidarDeg] = distance;
	if (isFront(lidarDeg)) {
		const int stopDist = 300;  // mm: これ以下は停止
		const int slowDist = 1200; // mm: ここから先は最大速度
		distance = clampi(distance, 0, 5000);

		int target = 0;
		if (distance <= stopDist) {
			target = 0;
		} else if (distance >= slowDist) {
			target = 255;
		} else {
			float t = float(distance - stopDist) / float(slowDist - stopDist);
			target = int(t * 255.0f);
		}

		// 平滑化（指数移動平均）
		_speed = (_speed * 7 + target * 3) / 10;
	}
}

String Drive::info() {
	String res;
	res += String(_speed);
	res += ", ";
	res += String(_angle);
	return res;
}
