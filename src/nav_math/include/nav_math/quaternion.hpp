#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// nav_math/quaternion.hpp
//
// Cuaternión de Hamilton (w + xi + yj + zk) para estimación de orientación.
// Sin dependencias externas.
//
// Convención:
//   q representa la rotación FROM frame del cuerpo TO frame del mundo.
//   v_world = q.rotate(v_body)
// ─────────────────────────────────────────────────────────────────────────────

#include "nav_math/constants.hpp"
#include "nav_math/vec3.hpp"
#include <array>
#include <cmath>

namespace nav_math {

struct Quaternion {
    float w{1.f};
    float x{0.f};
    float y{0.f};
    float z{0.f};

    static constexpr Quaternion identity() noexcept { return {1.f, 0.f, 0.f, 0.f}; }

    // Producto de Hamilton: this ⊗ other (composición de rotaciones)
    Quaternion operator*(const Quaternion& o) const noexcept {
        return {
            w * o.w - x * o.x - y * o.y - z * o.z,
            w * o.x + x * o.w + y * o.z - z * o.y,
            w * o.y - x * o.z + y * o.w + z * o.x,
            w * o.z + x * o.y - y * o.x + z * o.w
        };
    }

    Quaternion conjugate() const noexcept { return {w, -x, -y, -z}; }

    float norm2() const noexcept { return w * w + x * x + y * y + z * z; }

    Quaternion normalized() const noexcept {
        const float n = std::sqrt(norm2());
        if (n < 1e-9f) return identity();
        const float inv = 1.f / n;
        return {w * inv, x * inv, y * inv, z * inv};
    }

    // Rota un vector del frame del cuerpo al frame del mundo: q ⊗ [0,v] ⊗ q*
    // Optimización algebraica: v' = v + 2w*(q_vec × v) + 2*(q_vec × (q_vec × v))
    Vec3 rotate(const Vec3& v) const noexcept {
        const Vec3 qv{x, y, z};
        const Vec3 t = qv.cross(v) * 2.f;
        return v + t * w + qv.cross(t);
    }

    // Interpolación esférica lineal (camino más corto)
    static Quaternion slerp(const Quaternion& a, const Quaternion& b, float t) noexcept {
        float dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
        Quaternion b2 = (dot < 0.f) ? Quaternion{-b.w, -b.x, -b.y, -b.z} : b;
        if (dot < 0.f) dot = -dot;
        if (dot > 0.9995f) {
            return Quaternion{a.w + t * (b2.w - a.w),
                              a.x + t * (b2.x - a.x),
                              a.y + t * (b2.y - a.y),
                              a.z + t * (b2.z - a.z)}.normalized();
        }
        const float theta0 = std::acos(dot);
        const float theta  = theta0 * t;
        const float s0 = std::cos(theta) - dot * std::sin(theta) / std::sin(theta0);
        const float s1 = std::sin(theta) / std::sin(theta0);
        return Quaternion{s0 * a.w + s1 * b2.w,
                          s0 * a.x + s1 * b2.x,
                          s0 * a.y + s1 * b2.y,
                          s0 * a.z + s1 * b2.z}.normalized();
    }

    // Construye un cuaternión de rotación eje-ángulo
    static Quaternion fromAngleAxis(float angle_rad, const Vec3& axis) noexcept {
        const float ha = angle_rad * 0.5f;
        const float s  = std::sin(ha);
        const Vec3  na = axis.normalized();
        return {std::cos(ha), na.x * s, na.y * s, na.z * s};
    }

    // Construye dq a partir del vector de rotación omega*dt (mapa exponencial exacto en SO(3)).
    // Para ángulos pequeños: dq ≈ [1, ω·dt/2]. Para ángulos grandes: exacto.
    static Quaternion fromOmegaDt(const Vec3& omega_dt) noexcept {
        const float angle = omega_dt.norm();
        if (angle < 1e-9f) return identity();
        const float ha = angle * 0.5f;
        const float s  = std::sin(ha) / angle;
        return {std::cos(ha), omega_dt.x * s, omega_dt.y * s, omega_dt.z * s};
    }

    // Extrae el ángulo de yaw (rotación alrededor del eje Y en el frame del mundo, Y-abajo).
    // Usa la descomposición ZXY: no tiene singularidad en gimbal lock del eje Y.
    float yawY() const noexcept {
        const float siny = 2.f * (w * y + x * z);
        const float cosy = 1.f - 2.f * (x * x + y * y);
        return std::atan2(siny, cosy);
    }

    // Extrae los ángulos de Euler en convención YXZ (yaw-pitch-roll con Y vertical).
    // Retorna {yaw_rad, pitch_rad, roll_rad}.
    std::array<float, 3> toYPR() const noexcept {
        const float yaw   = yawY();
        const float sinp  = 2.f * (w * x - y * z);
        const float pitch = std::abs(sinp) >= 1.f
                            ? std::copysign(kHalfPi, sinp)
                            : std::asin(sinp);
        const float sinr  = 2.f * (w * z + x * y);
        const float cosr  = 1.f - 2.f * (x * x + z * z);
        const float roll  = std::atan2(sinr, cosr);
        return {yaw, pitch, roll};
    }

    // Matriz de rotación 3×3 en orden fila-mayor (row-major).
    std::array<float, 9> toRotMat() const noexcept {
        const float ww = w * w, xx = x * x, yy = y * y, zz = z * z;
        const float wx = w * x, wy = w * y, wz = w * z;
        const float xy = x * y, xz = x * z, yz = y * z;
        return {ww + xx - yy - zz,  2.f * (xy - wz),      2.f * (xz + wy),
                2.f * (xy + wz),    ww - xx + yy - zz,    2.f * (yz - wx),
                2.f * (xz - wy),    2.f * (yz + wx),      ww - xx - yy + zz};
    }
};

}  // namespace nav_math
