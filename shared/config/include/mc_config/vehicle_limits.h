#pragma once

#include <cstdint>

namespace mc_config {
inline constexpr float kSteerAngleMinDeg = -25.0f;
inline constexpr float kSteerAngleMaxDeg = 25.0f;
inline constexpr int kSteerAngleMinCdeg = -2500;
inline constexpr int kSteerAngleMaxCdeg = 2500;
inline constexpr float kSteerCdegScale = 100.0f;
} // namespace mc_config
