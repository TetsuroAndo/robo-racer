#include "uart_posix.h"
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

static speed_t toSpeed(int baud) {
	switch (baud) {
	case 115200:
		return B115200;
	case 230400:
		return B230400;
	case 460800:
		return B460800;
	case 921600:
		return B921600;
	default:
		return B921600;
	}
}

bool UartPosix::open(const std::string &dev, int baud) {
	_fd = ::open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (_fd < 0)
		return false;

	termios tio{};
	if (tcgetattr(_fd, &tio) != 0)
		return false;

	cfmakeraw(&tio);
	tio.c_cflag |= (CLOCAL | CREAD);
	tio.c_cflag &= ~CRTSCTS;
	tio.c_cflag &= ~CSTOPB;
	tio.c_cflag &= ~PARENB;
	tio.c_cflag &= ~CSIZE;
	tio.c_cflag |= CS8;

	speed_t sp = toSpeed(baud);
	cfsetispeed(&tio, sp);
	cfsetospeed(&tio, sp);

	tio.c_cc[VTIME] = 0;
	tio.c_cc[VMIN] = 0;

	if (tcsetattr(_fd, TCSANOW, &tio) != 0)
		return false;

	return true;
}

void UartPosix::close() {
	if (_fd >= 0)
		::close(_fd);
	_fd = -1;
}

int UartPosix::readSome(void *buf, int cap) {
	if (_fd < 0)
		return -1;
	int n = (int)::read(_fd, buf, (size_t)cap);
	if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
		return 0;
	return n;
}

int UartPosix::writeAll(const void *buf, int len) {
	if (_fd < 0)
		return -1;
	const uint8_t *p = (const uint8_t *)buf;
	int off = 0;
	while (off < len) {
		int n = (int)::write(_fd, p + off, (size_t)(len - off));
		if (n < 0) {
			if (errno == EAGAIN || errno == EWOULDBLOCK)
				continue;
			return -1;
		}
		off += n;
	}
	return off;
}
