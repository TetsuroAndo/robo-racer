#pragma once
#include <cstdint>
#include <string>

namespace mc::serial {

class Uart {
public:
	Uart() = default;
	~Uart();

	bool open(const std::string &dev, int baud);
	void close();

	int fd() const { return _fd; }

	int read(uint8_t *buf, int cap);
	int write(const uint8_t *buf, int len);

private:
	int _fd = -1;
};

} // namespace mc::serial
