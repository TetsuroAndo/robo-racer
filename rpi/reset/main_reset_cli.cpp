#include "Esp32Reset.h"

#include <exception>
#include <iostream>

int main() {
	try {
		Esp32Reset reset;
		reset.pulse();
		std::cout << "OK: ESP32 reset pulsed\n";
		return 0;
	} catch (const std::exception &e) {
		std::cerr << "ERR: " << e.what() << "\n";
		return 1;
	}
}
