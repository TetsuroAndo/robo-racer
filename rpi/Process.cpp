#include "Process.h"

#include <cmath>

Process::Process() {}

Process::~Process() {}

ProcResult Process::proc(const std::vector< LidarData > &lidarData) const {
	(void)lidarData;
	static float t = 0.0f;
	const float dt = 0.1f;

	const int speed = static_cast< int >((std::sin(t) + 1.0f) * 0.5f * 255.0f);
	const int angle = static_cast< int >(std::sin(t * 0.7f) * 30.0f);

	t += dt;
	if (t > 2.0f * 3.14159265f) {
		t -= 2.0f * 3.14159265f;
	}

	return ProcResult(speed, angle);
}
