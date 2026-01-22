#include "mc_core/Process.h"
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

namespace mc {

pid_t Process::spawn(const std::string &path,
					 const std::vector< std::string > &args) {
	pid_t pid = fork();
	if (pid != 0)
		return pid;

	std::vector< char * > argv;
	argv.reserve(args.size() + 2);
	argv.push_back(const_cast< char * >(path.c_str()));
	for (const auto &a : args)
		argv.push_back(const_cast< char * >(a.c_str()));
	argv.push_back(nullptr);

	execv(path.c_str(), argv.data());
	_exit(127);
}

bool Process::isRunning(pid_t pid) {
	if (pid <= 0)
		return false;
	return kill(pid, 0) == 0;
}

void Process::terminate(pid_t pid) {
	if (pid <= 0)
		return;
	kill(pid, SIGTERM);
	waitpid(pid, nullptr, 0);
}

} // namespace mc
