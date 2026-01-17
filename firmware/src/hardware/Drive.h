#pragma once

#include "Engine.h"
#include "Steer.h"

class Drive {
public:
	Drive();
	~Drive();

	void begin();
	void control();
	void evalInput(int lidarDeg, int distance);
private:
	Engine _engine;
	Steer _steer;

	int _speed;
	int _angle;
};
