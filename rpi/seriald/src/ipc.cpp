#include "ipc.h"
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

bool IpcServer::open(const std::string &path) {
	_path = path;
	_fd = ::socket(AF_UNIX, SOCK_DGRAM, 0);
	if (_fd < 0)
		return false;

	::unlink(path.c_str());

	sockaddr_un addr{};
	addr.sun_family = AF_UNIX;
	strncpy(addr.sun_path, path.c_str(), sizeof(addr.sun_path) - 1);

	if (::bind(_fd, (sockaddr *)&addr, sizeof(addr)) != 0)
		return false;
	::fcntl(_fd, F_SETFL, O_NONBLOCK);
	return true;
}

void IpcServer::close() {
	if (_fd >= 0)
		::close(_fd);
	_fd = -1;
	if (!_path.empty())
		::unlink(_path.c_str());
}

int IpcServer::recvSome(void *buf, int cap) {
	if (_fd < 0)
		return -1;
	int n = (int)::recv(_fd, buf, (size_t)cap, 0);
	if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
		return 0;
	return n;
}
