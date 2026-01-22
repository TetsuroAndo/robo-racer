#pragma once
#include <string>
#include <sys/types.h>
#include <vector>

namespace mc {

class Process {
public:
	static pid_t spawn(const std::string& path, const std::vector<std::string>& args);
	static bool isRunning(pid_t pid);
	static void terminate(pid_t pid);
};

} // namespace mc
