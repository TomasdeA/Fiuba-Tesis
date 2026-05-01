#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// nav_odometry/iodometry_backend.hpp
//
// Interfaz abstracta para backends de odometría intercambiables.
// Permite sustituir el backend (complementary filter, OpenVINS, ORB-SLAM3, etc.)
// sin modificar el pipeline de adquisición ni el nodo ROS2.
// ─────────────────────────────────────────────────────────────────────────────

#include "nav_odometry/types.hpp"

namespace nav_odometry {

class IOdometryBackend {
public:
    virtual ~IOdometryBackend() = default;

    // Procesa una muestra de IMU (gyro + accel combinados o separados).
    // Medido en D435i: ~200 Hz para gyro / ~100 Hz para accel.
    virtual void processImu(const ImuSample& sample) = 0;

    // Procesa un frame visual (color + depth) a ~30 fps.
    virtual void processRgbd(const ColorFrame& frame) = 0;

    // Retorna el estado de odometría más reciente.
    // Puede llamarse desde cualquier hilo; la implementación debe ser thread-safe.
    virtual OdometryState getState() const = 0;

    // Reinicia el estado (posición, orientación, velocidad) a cero.
    virtual void reset() = 0;

    // Configura la intrínseca de la cámara izquierda y el baseline estéreo.
    // Debe llamarse antes de processRgbd().
    virtual void setCameraIntrinsics(const CameraIntrinsics& intrinsics) = 0;

    // Nombre del backend (para logging y benchmark).
    virtual const char* name() const noexcept = 0;
};

}  // namespace nav_odometry
