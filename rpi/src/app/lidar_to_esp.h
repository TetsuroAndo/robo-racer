#pragma once

#include "config/Params.h"

int run_lidar_to_esp(const char *lidar_dev, int lidar_baud,
					 const char *esp_dev, const cfg::Params &params,
					 const char *planner_name = "ftg");
