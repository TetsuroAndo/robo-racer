#pragma once

class Sender {
	Sender(const char* esp_dev);
	~Sender();

	void send(int speed, int angle);
private:
	void _init(const char* esp_dev);
	int _espFd;
};
