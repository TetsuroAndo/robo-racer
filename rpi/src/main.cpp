#include "app/lidar_to_esp.h"
#include "config/Config.h"
#include "config/Params.h"

#include <cstdlib>
#include <cstring>

static const char *extract_arg_value(const char *arg, const char *key) {
	if (std::strncmp(arg, key, std::strlen(key)) == 0) {
		return arg + std::strlen(key);
	}
	return nullptr;
}

static bool parse_int(const char *value, int &out) {
	if (!value)
		return false;
	char *end = nullptr;
	long v = std::strtol(value, &end, 10);
	if (!end || end == value)
		return false;
	out = static_cast< int >(v);
	return true;
}

static bool parse_float(const char *value, float &out) {
	if (!value)
		return false;
	char *end = nullptr;
	float v = std::strtof(value, &end);
	if (!end || end == value)
		return false;
	out = v;
	return true;
}

int main(int argc, char **argv) {
	const char *lidar_dev = cfg::DEFAULT_LIDAR_DEVICE;
	int lidar_baud = cfg::DEFAULT_LIDAR_BAUD;
	const char *seriald_sock = cfg::DEFAULT_SERIALD_SOCK;
	const char *planner = "ftg";
	cfg::Params params;

	for (int i = 1; i < argc; ++i) {
		if (const char *value = extract_arg_value(argv[i], "--lidar=")) {
			lidar_dev = value;
			continue;
		}
		if (const char *value = extract_arg_value(argv[i], "--baud=")) {
			parse_int(value, lidar_baud);
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

		int int_value = 0;
		float float_value = 0.0f;
		if (const char *value = extract_arg_value(argv[i], "--fov-min=")) {
			if (parse_float(value, float_value))
				params.fov_min_deg = float_value;
			continue;
		}
		if (const char *value = extract_arg_value(argv[i], "--fov-max=")) {
			if (parse_float(value, float_value))
				params.fov_max_deg = float_value;
			continue;
		}
		if (const char *value = extract_arg_value(argv[i], "--angle-step=")) {
			if (parse_float(value, float_value))
				params.angle_step_deg = float_value;
			continue;
		}
		if (const char *value =
				extract_arg_value(argv[i], "--vehicle-width=")) {
			if (parse_int(value, int_value))
				params.vehicle_width_mm = int_value;
			continue;
		}
		if (const char *value =
				extract_arg_value(argv[i], "--safety-margin=")) {
			if (parse_int(value, int_value))
				params.safety_margin_mm = int_value;
			continue;
		}
		if (const char *value =
				extract_arg_value(argv[i], "--free-threshold=")) {
			if (parse_int(value, int_value))
				params.free_threshold_mm = int_value;
			continue;
		}
		if (const char *value =
				extract_arg_value(argv[i], "--stop-distance=")) {
			if (parse_int(value, int_value))
				params.stop_distance_mm = int_value;
			continue;
		}
		if (const char *value = extract_arg_value(argv[i], "--front-sector=")) {
			if (parse_int(value, int_value))
				params.front_sector_deg = int_value;
			continue;
		}
		if (const char *value =
				extract_arg_value(argv[i], "--bubble-radius=")) {
			if (parse_int(value, int_value))
				params.bubble_radius_mm = int_value;
			continue;
		}
		if (const char *value =
				extract_arg_value(argv[i], "--disparity-threshold=")) {
			if (parse_int(value, int_value))
				params.disparity_threshold_mm = int_value;
			continue;
		}
		if (const char *value =
				extract_arg_value(argv[i], "--smoothing-window=")) {
			if (parse_int(value, int_value))
				params.smoothing_window = int_value;
			continue;
		}
		if (const char *value =
				extract_arg_value(argv[i], "--steer-slowdown-gain=")) {
			if (parse_float(value, float_value))
				params.steer_slowdown_gain = float_value;
			continue;
		}
		if (const char *value =
				extract_arg_value(argv[i], "--steer-slowdown-angle=")) {
			if (parse_float(value, float_value))
				params.steer_slowdown_angle_deg = float_value;
			continue;
		}
		if (const char *value = extract_arg_value(argv[i], "--steer-beta=")) {
			if (parse_float(value, float_value))
				params.steer_smooth_beta = float_value;
			continue;
		}
		if (const char *value = extract_arg_value(argv[i], "--steer-rate=")) {
			if (parse_int(value, int_value))
				params.steer_rate_limit_deg = int_value;
			continue;
		}
		if (const char *value = extract_arg_value(argv[i], "--speed-gain=")) {
			if (parse_float(value, float_value))
				params.speed_gain = float_value;
			continue;
		}
		if (const char *value = extract_arg_value(argv[i], "--accel-limit=")) {
			if (parse_float(value, float_value))
				params.accel_limit = float_value;
			continue;
		}
	}

	return run_lidar_to_esp(lidar_dev, lidar_baud, seriald_sock, params,
							planner);
}
