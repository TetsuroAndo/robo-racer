#pragma once
#include <string>
#include <unordered_set>

namespace mc::ipc {

class DgramServer {
public:
	~DgramServer();

	bool bind(const std::string& sock_path);
	void close();

	int fd() const { return _fd; }
	const std::string& path() const { return _path; }

	int recv(void* buf, int cap, std::string* from_path);
	int sendTo(const std::string& path, const void* buf, int len);
	int broadcast(const void* buf, int len);

private:
	int _fd = -1;
	std::string _path;
	std::unordered_set<std::string> _peers;
};

class DgramClient {
public:
	~DgramClient();

	bool connect(const std::string& server_path);
	void close();

	int fd() const { return _fd; }
	const std::string& localPath() const { return _local_path; }

	int send(const void* buf, int len);
	int recv(void* buf, int cap);

private:
	int _fd = -1;
	std::string _server_path;
	std::string _local_path;
};

} // namespace mc::ipc
