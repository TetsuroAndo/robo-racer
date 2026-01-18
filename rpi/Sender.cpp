#include "Sender.h"
#include "uart.h"
#include <cstdlib>
#include <iostream>
#include <unistd.h>

Sender::Sender(const char *esp_dev) { _init(esp_dev); }

Sender::~Sender() { close(_espFd); }

void Sender::send(int speed, int angle) {
	char line[32];
	int n = std::snprintf(line, sizeof(line), "%u,%u\n", speed, angle);
	if (n > 0) {
		write(_espFd, line, (size_t)n);
		std::cerr << line;
	}
}

void Sender::_init(const char *esp_dev) {
	_espFd = uart_open_writeonly(esp_dev, 115200);
	if (_espFd < 0) {
		exit(1);
	}
}
