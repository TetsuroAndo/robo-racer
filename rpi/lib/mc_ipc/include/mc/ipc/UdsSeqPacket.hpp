#pragma once
#include <string>
#include <sys/socket.h>
#include <vector>

#ifdef __APPLE__
#define MC_IPC_DEFAULT_SOCK_TYPE SOCK_DGRAM
#else
#define MC_IPC_DEFAULT_SOCK_TYPE SOCK_SEQPACKET
#endif

namespace mc::ipc {

class UdsServer {
public:
	explicit UdsServer(int sock_type = MC_IPC_DEFAULT_SOCK_TYPE)
		: sock_type_(sock_type) {}
	~UdsServer();

	bool listen(const std::string &path);
	void close();

	int fd() const { return fd_; }
	int sock_type() const { return sock_type_; }
	std::string path() const { return path_; }

	int accept_client();
	void remove_client(int cfd);

	const std::vector< int > &clients() const { return clients_; }

private:
	int fd_{-1};
	int sock_type_{MC_IPC_DEFAULT_SOCK_TYPE};
	std::string path_;
	std::vector< int > clients_;
};

class UdsClient {
public:
	explicit UdsClient(int sock_type = MC_IPC_DEFAULT_SOCK_TYPE)
		: sock_type_(sock_type) {}
	~UdsClient();

	bool connect(const std::string &path);
	void close();

	int fd() const { return fd_; }

private:
	int fd_{-1};
	int sock_type_{MC_IPC_DEFAULT_SOCK_TYPE};
	std::string bound_path_;
};

} // namespace mc::ipc
