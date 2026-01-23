#pragma once

int run_lidar_to_esp(const char *lidar_dev, int lidar_baud,
				 const char *esp_dev, const char *planner_name = "ftg");
