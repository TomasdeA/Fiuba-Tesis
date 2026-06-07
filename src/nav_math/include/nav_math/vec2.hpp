#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// nav_math/vec2.hpp
//
// Vector bidimensional de precisión simple para el plano horizontal XZ.
//
// Convención de coordenadas del proyecto:
//   X = derecha, Z = adelante.
// ─────────────────────────────────────────────────────────────────────────────

#include <cmath>

namespace nav_math {

struct Vec2 {
    float x{0.f};
    float z{0.f};

    Vec2 operator+(const Vec2& o) const noexcept { return {x + o.x, z + o.z}; }
    Vec2 operator-(const Vec2& o) const noexcept { return {x - o.x, z - o.z}; }
    Vec2 operator*(float s)       const noexcept { return {x * s,   z * s};   }

    Vec2& operator+=(const Vec2& o) noexcept { x += o.x; z += o.z; return *this; }
    Vec2& operator-=(const Vec2& o) noexcept { x -= o.x; z -= o.z; return *this; }

    float dot(const Vec2& o) const noexcept { return x * o.x + z * o.z; }

    float norm2() const noexcept { return x * x + z * z; }
    float norm()  const noexcept { return std::sqrt(norm2()); }

    Vec2 normalized() const noexcept {
        const float n = norm();
        return (n > 1e-9f) ? (*this) * (1.f / n) : Vec2{};
    }
};

}  // namespace nav_math
