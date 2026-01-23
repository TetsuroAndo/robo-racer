#pragma once

#include <type_traits>

namespace mc::common {

template <typename T>
constexpr T clamp(T value, T lo, T hi) {
  static_assert(std::is_arithmetic_v<T>, "clamp requires arithmetic types");
  if (value < lo)
    return lo;
  if (value > hi)
    return hi;
  return value;
}

} // namespace mc::common
