#include <mc/ipc_dgram.h>

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

namespace mc::ipc {

static bool set_nonblock(int fd) {
	int fl = fcntl(fd, F_GETFL, 0);
	if (fl < 0)
		return false;
	return fcntl(fd, F_SETFL, fl | O_NONBLOCK) == 0;
}

DgramServer::~DgramServer() { close(); }

bool DgramServer::bind(const std::string &sock_path) {
	close();
	_fd = ::socket(AF_UNIX, SOCK_DGRAM, 0);
	if (_fd < 0)
		return false;
	if (!set_nonblock(_fd)) {
		close();
		return false;
	}

	::unlink(sock_path.c_str());

	sockaddr_un addr{};
	addr.sun_family = AF_UNIX;
	snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", sock_path.c_str());
	if (::bind(_fd, (sockaddr *)&addr, sizeof(addr)) != 0) {
		close();
		return false;
	}
	_path = sock_path;
	return true;
}

void DgramServer::close() {
	if (_fd >= 0) {
		::close(_fd);
		_fd = -1;
	}
	if (!_path.empty())
		::unlink(_path.c_str());
	_path.clear();
}

int DgramServer::recv(void *buf, int cap, std::string *from_path) {
	if (_fd < 0)
		return -1;
	sockaddr_un src{};
	socklen_t sl = sizeof(src);
	int n = (int)::recvfrom(_fd, buf, (size_t)cap, 0, (sockaddr *)&src, &sl);
	if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
		return 0;
	if (n <= 0)
		return n;
	std::string src_path = src.sun_path;
	if (!src_path.empty())
		_peers.insert(src_path);
	if (from_path)
		*from_path = src_path;
	return n;
}

int DgramServer::sendTo(const std::string &path, const void *buf, int len) {
	if (_fd < 0)
		return -1;
	sockaddr_un dst{};
	dst.sun_family = AF_UNIX;
	snprintf(dst.sun_path, sizeof(dst.sun_path), "%s", path.c_str());
	return (int)::sendto(_fd, buf, (size_t)len, 0, (sockaddr *)&dst,
						 sizeof(dst));
}

int DgramServer::broadcast(const void *buf, int len) {
	if (_fd < 0)
		return -1;
	int sent = 0;
	for (const auto &peer : _peers) {
		int r = sendTo(peer, buf, len);
		if (r >= 0)
			sent++;
	}
	return sent;
}

DgramClient::~DgramClient() { close(); }

bool DgramClient::connect(const std::string &server_path) {
	close();
	_fd = ::socket(AF_UNIX, SOCK_DGRAM, 0);
	if (_fd < 0)
		return false;
	if (!set_nonblock(_fd)) {
		close();
		return false;
	}

	_local_path = "/tmp/seriald_client_" + std::to_string(getpid()) + "_" +
				  std::to_string((uintptr_t)this) + ".sock";
	::unlink(_local_path.c_str());

	sockaddr_un local{};
	local.sun_family = AF_UNIX;
	snprintf(local.sun_path, sizeof(local.sun_path), "%s", _local_path.c_str());
	if (::bind(_fd, (sockaddr *)&local, sizeof(local)) != 0) {
		close();
		return false;
	}

	_server_path = server_path;
	return true;
}

void DgramClient::close() {
	if (_fd >= 0) {
		::close(_fd);
		_fd = -1;
	}
	if (!_local_path.empty())
		::unlink(_local_path.c_str());
	_local_path.clear();
	_server_path.clear();
}

int DgramClient::send(const void *buf, int len) {
	if (_fd < 0)
		return -1;
	sockaddr_un dst{};
	dst.sun_family = AF_UNIX;
	snprintf(dst.sun_path, sizeof(dst.sun_path), "%s", _server_path.c_str());
	return (int)::sendto(_fd, buf, (size_t)len, 0, (sockaddr *)&dst,
						 sizeof(dst));
}

int DgramClient::recv(void *buf, int cap) {
	if (_fd < 0)
		return -1;
	int n = (int)::recv(_fd, buf, (size_t)cap, 0);
	if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
		return 0;
	return n;
}

} // namespace mc::ipc
