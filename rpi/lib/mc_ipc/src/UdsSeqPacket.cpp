#include <mc/core/Log.hpp>
#include <mc/ipc/UdsSeqPacket.hpp>

#include <errno.h>
#include <string.h>

#include <fcntl.h>
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

UdsServer::~UdsServer() { close(); }

bool UdsServer::listen(const std::string &path) {
	close();
	path_ = path;

	fd_ = ::socket(AF_UNIX, SOCK_SEQPACKET, 0);
	if (fd_ < 0) {
		MC_LOGE("uds", "socket failed: " + std::string(std::strerror(errno)));
		return false;
	}
	set_nonblock(fd_);

	::unlink(path.c_str());

	sockaddr_un addr{};
	addr.sun_family = AF_UNIX;
	snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", path.c_str());

	if (::bind(fd_, (sockaddr *)&addr, sizeof(addr)) != 0) {
		MC_LOGE("uds", "bind failed: " + std::string(std::strerror(errno)));
		close();
		return false;
	}

	if (::listen(fd_, 16) != 0) {
		MC_LOGE("uds", "listen failed: " + std::string(std::strerror(errno)));
		close();
		return false;
	}

	return true;
}

void UdsServer::close() {
	for (int c : clients_)
		::close(c);
	clients_.clear();
	if (fd_ >= 0) {
		::close(fd_);
		fd_ = -1;
	}
	if (!path_.empty()) {
		::unlink(path_.c_str());
		path_.clear();
	}
}

int UdsServer::accept_client() {
	if (fd_ < 0)
		return -1;
	int cfd = ::accept(fd_, nullptr, nullptr);
	if (cfd < 0) {
		if (errno == EAGAIN || errno == EWOULDBLOCK)
			return -1;
		MC_LOGE("uds", "accept failed: " + std::string(std::strerror(errno)));
		return -1;
	}
	set_nonblock(cfd);
	clients_.push_back(cfd);
	return cfd;
}

void UdsServer::remove_client(int cfd) {
	for (size_t i = 0; i < clients_.size(); ++i) {
		if (clients_[i] == cfd) {
			::close(cfd);
			clients_.erase(clients_.begin() + (long)i);
			return;
		}
	}
}

UdsClient::~UdsClient() { close(); }

bool UdsClient::connect(const std::string &path) {
	close();

	fd_ = ::socket(AF_UNIX, SOCK_SEQPACKET, 0);
	if (fd_ < 0) {
		MC_LOGE("uds", "socket failed: " + std::string(std::strerror(errno)));
		return false;
	}

	sockaddr_un addr{};
	addr.sun_family = AF_UNIX;
	snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", path.c_str());

	if (::connect(fd_, (sockaddr *)&addr, sizeof(addr)) != 0) {
		MC_LOGE("uds", "connect failed: " + std::string(std::strerror(errno)));
		close();
		return false;
	}
	return true;
}

void UdsClient::close() {
	if (fd_ >= 0) {
		::close(fd_);
		fd_ = -1;
	}
}

} // namespace mc::ipc
