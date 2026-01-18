#pragma once

#include "Engine.h"
#include "Steer.h"
#include <map>

class Drive {
public:
	Drive();
	~Drive();

	void begin();
	void control();

	void setSpeed(int newSpeed);
	void setAngle(int newAngle);

	void updateTimeout();
	bool evalTimeout();

	String info();
private:
	Engine _engine;
	Steer _steer;

	int _speed;
	float _angle;
	std::map<int, int> _angles;
	unsigned long _lastUpdate;
};
