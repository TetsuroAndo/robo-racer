#pragma once
#include <string>

class UartPosix {
public:
	bool open(const std::string& dev, int baud);
	void close();

	int fd() const { return _fd; }

	int readSome(void* buf, int cap);
	int writeAll(const void* buf, int len);

private:
	int _fd = -1;
};
