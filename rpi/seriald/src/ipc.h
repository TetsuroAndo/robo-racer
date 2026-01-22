#pragma once
#include <string>

class IpcServer {
public:
	bool open(const std::string& path);
	void close();
	int fd() const { return _fd; }
	int recvSome(void* buf, int cap);

private:
	int _fd = -1;
	std::string _path;
};
