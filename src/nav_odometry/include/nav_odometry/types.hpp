#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// nav_odometry/types.hpp
//
// Tipos del pipeline VIO: Pose, OdometryState y estructuras de mensajes de
// sensores sin dependencias de ROS2.
//
// Vec3 y Quaternion provienen del paquete compartido nav_math.
//
// Convención de coordenadas interna (consistente con local_mapper):
//   Frame del cuerpo (cámara óptica D435i): X derecha, Y abajo, Z adelante.
//   Frame del mundo (gravity-aligned):      Y abajo (dirección de la gravedad).
//   La gravedad en el frame del mundo es g_world = {0, +g, 0}.
//
// La salida al topic nav_odom se convierte a REP-103 (Z arriba) antes de
// publicar (ver odometry_node.cpp).
// ─────────────────────────────────────────────────────────────────────────────

#include <nav_math/nav_math.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

namespace nav_odometry {

// Vec3 y Quaternion se importan de nav_math para evitar duplicación.
using nav_math::Vec3;
using nav_math::Quaternion;

// ─── Pose ─────────────────────────────────────────────────────────────────────
struct Pose {
    Vec3       position{};
    Quaternion orientation = Quaternion::identity();
};

// ─── Covarianza 6x6 (posición + orientación) ─────────────────────────────────
using Cov6 = std::array<double, 36>;

// ─── Estado de odometría ──────────────────────────────────────────────────────
struct OdometryState {
    double     timestamp_s{0.0};
    Pose       pose;
    Vec3       linear_velocity{};
    Vec3       angular_velocity{};
    Cov6       pose_covariance{};
    Cov6       twist_covariance{};
    bool       valid{false};
};

// ─── Muestra de IMU ──────────────────────────────────────────────────────────
struct ImuSample {
    double timestamp_s{0.0};
    Vec3   gyro{};   // velocidad angular [rad/s], frame del cuerpo
    Vec3   accel{};  // aceleración específica [m/s^2], frame del cuerpo
};

// ─── Frame visual color (sin copia de datos: vista sobre buffer ROS2) ────────
// Imagen color convertida a mono 8-bit. La profundidad se obtiene via DepthFrame.
struct ColorFrame {
    double          timestamp_s{0.0};
    uint32_t        width{0};
    uint32_t        height{0};
    const uint8_t*  data{nullptr};        // imagen color convertida a mono 8-bit
    uint32_t        step{0};              // bytes por fila
};

// ─── Imagen de profundidad de hardware (sin copia de datos) ──────────────────
// Vista no propietaria sobre el buffer de profundidad ROS2.
// Para D435i: depth/image_rect_raw es válido directamente con la intrínseca
// de infra1. Verificado con:
//   ros2 topic echo /camera/camera/extrinsics/depth_to_infra1 --once
// Resultado: rotation=identidad, translation=[0,0,0].
// El sensor de profundidad y la cámara IR izquierda comparten el mismo frame
// óptico en la D435i; no se requiere reproyección.
struct DepthFrame {
    const uint16_t* data{nullptr};   // valores crudos de profundidad (16-bit)
    uint32_t        width{0};
    uint32_t        height{0};
    uint32_t        step{0};         // bytes por fila
    float           scale_m{0.001f}; // raw * scale_m --> metros (D435i: 0.001)
};

// ─── Intrínseca de cámara (modelo pinhole) ────────────────────────────────────
struct CameraIntrinsics {
    float fx{0.f};
    float fy{0.f};
    float cx{0.f};
    float cy{0.f};
    uint32_t width{0};
    uint32_t height{0};
    bool valid{false};
};

// ─── Resultado de odometría visual ──────────────────────────────────────────
struct VisualOdometryResult {
    Quaternion delta_rotation = Quaternion::identity();  // dR en frame del cuerpo
    Vec3       delta_translation{};                      // dT en frame del cuerpo [m]
    int        num_inliers{0};
    float      confidence{0.f};  // [0,1]: calidad del resultado
    bool       valid{false};
};

// ─── Utilidades de ángulos ────────────────────────────────────────────────────
using nav_math::normalizeAngle;

}  // namespace nav_odometry
