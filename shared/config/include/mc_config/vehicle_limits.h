#pragma once

#include <cstdint>

namespace mc_config {
static constexpr float kSteerAngleMinDeg = -25.0f;
static constexpr float kSteerAngleMaxDeg = 25.0f;
static constexpr int kSteerAngleMinCdeg = -2500;
static constexpr int kSteerAngleMaxCdeg = 2500;
static constexpr float kSteerCdegScale = 100.0f;
} // namespace mc_config
