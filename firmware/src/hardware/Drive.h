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
	void evalInput(int lidarDeg, int distance);

	String info();
private:
	Engine _engine;
	Steer _steer;

	int _speed;
	float _angle;
	std::map<int, int> _angles;
};
