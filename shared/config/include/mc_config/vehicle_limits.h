#pragma once

#include <cstdint>

namespace mc_config {
static constexpr float STEER_ANGLE_MIN_DEG = -25.0f;
static constexpr float STEER_ANGLE_MAX_DEG = 25.0f;
static constexpr int STEER_ANGLE_MIN_CDEG = -2500;
static constexpr int STEER_ANGLE_MAX_CDEG = 2500;
static constexpr float STEER_CDEG_SCALE = 100.0f;
static constexpr int SPEED_INPUT_LIMIT = 255;
static constexpr int SPEED_MAX_MM_S = 5000; // 5m/s
} // namespace mc_config
