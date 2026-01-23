#pragma once

#include <stddef.h>
#include <stdint.h>

// Minimal stubs for host-side firmware tests.
class HardwareSerial {
public:
	int available() { return 0; }
	int read() { return -1; }
};

typedef void *QueueHandle_t;
typedef void *TaskHandle_t;
typedef int StaticQueue_t;
