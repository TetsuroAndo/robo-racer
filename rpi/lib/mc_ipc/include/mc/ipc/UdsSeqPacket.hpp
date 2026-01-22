#pragma once
#include <string>
#include <vector>

namespace mc::ipc {

class UdsServer {
public:
	~UdsServer();

	bool listen(const std::string &path);
	void close();

	int fd() const { return fd_; }
	std::string path() const { return path_; }

	int accept_client();
	void remove_client(int cfd);

	const std::vector< int > &clients() const { return clients_; }

private:
	int fd_{-1};
	std::string path_;
	std::vector< int > clients_;
};

class UdsClient {
public:
	~UdsClient();

	bool connect(const std::string &path);
	void close();

	int fd() const { return fd_; }

private:
	int fd_{-1};
};

} // namespace mc::ipc
