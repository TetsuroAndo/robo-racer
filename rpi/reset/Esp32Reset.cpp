#include "Esp32Reset.h"

#include <gpiod.h>
#include <stdexcept>
#include <thread>

Esp32Reset::Esp32Reset(Config cfg) : cfg_(cfg) {}

static void set_value(gpiod_line_request *req, unsigned int offset, int v) {
	if (gpiod_line_request_set_value(req, offset, v) < 0) {
		throw std::runtime_error("gpiod_line_request_set_value failed");
	}
}

void Esp32Reset::pulse() {
	gpiod_chip *chip = gpiod_chip_open(cfg_.chip_path.c_str());
	if (!chip) {
		throw std::runtime_error("gpiod_chip_open failed: " + cfg_.chip_path);
	}

	gpiod_line_settings *settings = gpiod_line_settings_new();
	gpiod_line_config *line_cfg = gpiod_line_config_new();
	gpiod_request_config *req_cfg = gpiod_request_config_new();
	if (!settings || !line_cfg || !req_cfg) {
		if (req_cfg) {
			gpiod_request_config_free(req_cfg);
		}
		if (line_cfg) {
			gpiod_line_config_free(line_cfg);
		}
		if (settings) {
			gpiod_line_settings_free(settings);
		}
		gpiod_chip_close(chip);
		throw std::runtime_error("gpiod config alloc failed");
	}

	gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);

	const int inactive = cfg_.assert_is_high ? 0 : 1;
	gpiod_line_settings_set_output_value(settings, inactive);

	const unsigned int offsets[] = {cfg_.bcm_gpio};
	if (gpiod_line_config_add_line_settings(line_cfg, offsets, 1, settings) <
		0) {
		gpiod_request_config_free(req_cfg);
		gpiod_line_config_free(line_cfg);
		gpiod_line_settings_free(settings);
		gpiod_chip_close(chip);
		throw std::runtime_error("gpiod_line_config_add_line_settings failed");
	}

	gpiod_request_config_set_consumer(req_cfg, "rpi-esp32-reset");

	gpiod_line_request *req = gpiod_chip_request_lines(chip, req_cfg, line_cfg);
	gpiod_request_config_free(req_cfg);
	gpiod_line_config_free(line_cfg);
	gpiod_line_settings_free(settings);

	if (!req) {
		gpiod_chip_close(chip);
		throw std::runtime_error("gpiod_chip_request_lines failed");
	}

	const int active = cfg_.assert_is_high ? 1 : 0;
	try {
		set_value(req, cfg_.bcm_gpio, active);
		std::this_thread::sleep_for(cfg_.pulse);
		set_value(req, cfg_.bcm_gpio, inactive);
		std::this_thread::sleep_for(cfg_.settle);
	} catch (...) {
		gpiod_line_request_release(req);
		gpiod_chip_close(chip);
		throw;
	}

	gpiod_line_request_release(req);
	gpiod_chip_close(chip);
}
