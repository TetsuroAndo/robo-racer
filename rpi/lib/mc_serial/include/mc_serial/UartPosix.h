#pragma once
#include <cstdint>
#include <string>

namespace mc::serial {

class UartPosix {
public:
	UartPosix() = default;
	~UartPosix();

	bool open(const std::string& dev, int baud);
	void close();

	int fd() const { return _fd; }

	int readSome(uint8_t* buf, int cap);
	bool writeAll(const uint8_t* buf, int len, int timeout_ms = 50);

private:
	int _fd = -1;
};

} // namespace mc::serial
