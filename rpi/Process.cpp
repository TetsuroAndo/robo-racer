#include "Process.h"

Process::~Process() {}

ProcResult Process::proc(const std::vector< LidarData > &lidarData) {
	return ProcResult(50, 0);
}
