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
	std::string cur;
	size_t i = 0;
	if (!path.empty() && path[0] == '/') {
		cur = "/";
		i = 1;
	}
	while (i <= path.size()) {
		const size_t next = path.find('/', i);
		const size_t end = (next == std::string::npos) ? path.size() : next;
		const std::string part = path.substr(i, end - i);
		i = (next == std::string::npos) ? path.size() + 1 : next + 1;
		if (part.empty() || part == ".")
			continue;
		if (!cur.empty() && cur.back() != '/')
			cur += "/";
		if (part == "..") {
			cur += part;
			continue;
		}
		cur += part;
		const int rc = mkdir(cur.c_str(), 0755);
		if (rc == 0 || errno == EEXIST)
			continue;
		const int err = errno;
		std::fprintf(stderr,
					 "ensure_dir: failed to create directory '%s': %s "
					 "(errno=%d)\n",
					 cur.c_str(), std::strerror(err), err);
		return;
	}
}

inline std::string dir_of(const std::string &path) {
	const size_t pos = path.find_last_of('/');
	if (pos == std::string::npos || pos == 0)
		return std::string();
	return path.substr(0, pos);
}

} // namespace mc::core
