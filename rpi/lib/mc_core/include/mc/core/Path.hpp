#pragma once

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <sys/stat.h>

namespace mc::core {

inline void ensure_dir(const std::string &path) {
	if (path.empty())
		return;
	const int rc = mkdir(path.c_str(), 0755);
	if (rc == 0 || errno == EEXIST)
		return;
	const int err = errno;
	std::fprintf(stderr,
				 "ensure_dir: failed to create directory '%s': %s (errno=%d)\n",
				 path.c_str(), std::strerror(err), err);
}

inline std::string dir_of(const std::string &path) {
	const size_t pos = path.find_last_of('/');
	if (pos == std::string::npos || pos == 0)
		return std::string();
	return path.substr(0, pos);
}

} // namespace mc::core
