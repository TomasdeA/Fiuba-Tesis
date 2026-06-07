#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// nav_math/constants.hpp
//
// Constantes y conversiones matemáticas comunes.
// ─────────────────────────────────────────────────────────────────────────────

#include <cmath>

namespace nav_math {

#ifdef M_PI
inline constexpr float kPi = static_cast<float>(M_PI);
#else
inline constexpr float kPi = 3.14159265358979323846f;
#endif

inline constexpr float kTwoPi = 2.0f * kPi;

#ifdef M_PI_2
inline constexpr float kHalfPi = static_cast<float>(M_PI_2);
#else
inline constexpr float kHalfPi = 0.5f * kPi;
#endif

#ifdef M_SQRT2
inline constexpr float kSqrt2 = static_cast<float>(M_SQRT2);
#else
inline constexpr float kSqrt2 = 1.41421356237309504880f;
#endif

inline constexpr float kHalfSqrt2 = 0.5f * kSqrt2;

constexpr float deg2rad(float degrees) noexcept {
    return degrees * kPi / 180.0f;
}

constexpr float rad2deg(float radians) noexcept {
    return radians * 180.0f / kPi;
}

}  // namespace nav_math
