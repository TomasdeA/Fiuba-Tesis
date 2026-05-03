#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// nav_math/vec3.hpp
//
// Vector tridimensional de precisión simple. Sin dependencias externas.
//
// Convención de coordenadas del proyecto (camera_depth_optical_frame):
//   X = derecha, Y = abajo, Z = adelante.
//   La gravedad en el frame del mundo es g_world = {0, +g, 0} (Y abajo).
// ─────────────────────────────────────────────────────────────────────────────

#include <cmath>

namespace nav_math {

struct Vec3 {
    float x{0.f};
    float y{0.f};
    float z{0.f};

    Vec3 operator+(const Vec3& o) const noexcept { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const noexcept { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(float s)       const noexcept { return {x * s,   y * s,   z * s};   }

    Vec3& operator+=(const Vec3& o) noexcept { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) noexcept { x -= o.x; y -= o.y; z -= o.z; return *this; }

    float dot(const Vec3& o) const noexcept { return x * o.x + y * o.y + z * o.z; }

    Vec3 cross(const Vec3& o) const noexcept {
        return {y * o.z - z * o.y,
                z * o.x - x * o.z,
                x * o.y - y * o.x};
    }

    float norm2() const noexcept { return x * x + y * y + z * z; }
    float norm()  const noexcept { return std::sqrt(norm2()); }

    Vec3 normalized() const noexcept {
        const float n = norm();
        return (n > 1e-9f) ? (*this) * (1.f / n) : Vec3{};
    }
};

}  // namespace nav_math
