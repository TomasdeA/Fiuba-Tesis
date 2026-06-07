#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// nav_math/nav_math.hpp
//
// Biblioteca común: incluye todos los tipos de nav_math.
// ─────────────────────────────────────────────────────────────────────────────

#include "nav_math/vec2.hpp"
#include "nav_math/vec3.hpp"
#include "nav_math/quaternion.hpp"

#include <cmath>

namespace nav_math {

// Normaliza un ángulo al rango (-π, π]
inline float normalizeAngle(float a) noexcept {
    constexpr float pi  = static_cast<float>(M_PI);
    constexpr float two_pi = 2.f * pi;
    while (a >  pi) a -= two_pi;
    while (a < -pi) a += two_pi;
    return a;
}

}  // namespace nav_math
