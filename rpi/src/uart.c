// uart.c
#include "uart.h"

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

static speed_t baud_to_speed(int baud) {
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
#ifdef B460800
        case 460800: return B460800;
#endif
#ifdef B921600
        case 921600: return B921600;
#endif
        default:     return 0;
    }
}

int uart_open_writeonly(const char *dev, int baud) {
    int fd = open(dev, O_WRONLY | O_NOCTTY);
    if (fd < 0) return -1;

    struct termios tio;
    if (tcgetattr(fd, &tio) != 0) { close(fd); return -1; }

    speed_t spd = baud_to_speed(baud);
    if (!spd) { close(fd); errno = EINVAL; return -1; }

    cfmakeraw(&tio);
    cfsetispeed(&tio, spd);
    cfsetospeed(&tio, spd);

    if (tcsetattr(fd, TCSANOW, &tio) != 0) { close(fd); return -1; }
    return fd;
}

int uart_open_readwrite(const char *dev, int baud) {
    int fd = open(dev, O_RDWR | O_NOCTTY);
    if (fd < 0) return -1;

    struct termios tio;
    if (tcgetattr(fd, &tio) != 0) { close(fd); return -1; }

    speed_t spd = baud_to_speed(baud);
    if (!spd) { close(fd); errno = EINVAL; return -1; }

    cfmakeraw(&tio);
    cfsetispeed(&tio, spd);
    cfsetospeed(&tio, spd);
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tio) != 0) { close(fd); return -1; }
    return fd;
}
