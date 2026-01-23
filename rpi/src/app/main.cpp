#include "app/lidar_to_esp.h"
#include "config/Config.h"

#include <cstdlib>
#include <cstring>

static const char *extract_arg_value(const char *arg, const char *key) {
	if (std::strncmp(arg, key, std::strlen(key)) == 0) {
		return arg + std::strlen(key);
	}
	return nullptr;
}

int main(int argc, char **argv) {
	const char *lidar_dev = cfg::DEFAULT_LIDAR_DEVICE;
	int lidar_baud = cfg::DEFAULT_LIDAR_BAUD;
	const char *seriald_sock = cfg::DEFAULT_SERIALD_SOCK;
	const char *planner = "ftg";

	for (int i = 1; i < argc; ++i) {
		if (const char *value = extract_arg_value(argv[i], "--lidar=")) {
			lidar_dev = value;
			continue;
		}
		if (const char *value = extract_arg_value(argv[i], "--baud=")) {
			lidar_baud = std::atoi(value);
			continue;
		}
		if (const char *value = extract_arg_value(argv[i], "--serial=")) {
			seriald_sock = value;
			continue;
		}
		if (const char *value = extract_arg_value(argv[i], "--planner=")) {
			planner = value;
			continue;
		}
	}

	return run_lidar_to_esp(lidar_dev, lidar_baud, seriald_sock, planner);
}
