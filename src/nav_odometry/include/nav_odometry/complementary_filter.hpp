#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// nav_odometry/complementary_filter.hpp
//
// Filtro complementario de Mahony para fusión IMU + visión estéreo.
// Corrige:
//   - Roll / pitch: mediante el vector de gravedad medido por el acelerómetro.
//   - Yaw: mediante la rotación relativa estimada por VO estéreo (alpha_yaw).
//
// Referencia: Mahony et al. (2008), "Nonlinear Complementary Filters on the
// Special Orthogonal Group", IEEE Trans. Automatic Control.
//
// Sin dependencias externas (C++17 puro).
// ─────────────────────────────────────────────────────────────────────────────

#include "nav_odometry/types.hpp"

namespace nav_odometry {

class ComplementaryFilter {
public:
    // ─── Configuración ────────────────────────────────────────────────────────
    struct Config {
        // Ganancia proporcional del feedback de acelerómetro (Mahony Kp).
        // Controla la velocidad de convergencia de pitch/roll.
        // Valor alto: rápido pero ruidoso. Bajo: suave pero lento.
        float kp_accel{2.0f};

        // Ganancia integral del feedback de acelerómetro (Mahony Ki).
        // Elimina el offset DC en la dirección de gravedad.
        float ki_accel{0.005f};

        // Ganancia de corrección de yaw por visión estéreo [0,1].
        // 0.0 = solo giroscopio (máx drift). 1.0 = solo visión (ruidoso).
        float alpha_yaw{0.1f};

        // Umbral mínimo de confianza de la VO para aplicar corrección de yaw.
        float min_visual_confidence{0.3f};

        // Módulo de gravedad esperado [m/s^2]. Muestras de accel fuera del rango
        // [g*(1-mag_tol), g*(1+mag_tol)] se ignoran para la corrección de tilt.
        float gravity_magnitude{9.807f};
        float gravity_mag_tolerance{0.3f};

        // Máximo dt aceptable [s]. Saltos mayores se ignoran.
        float max_dt_s{0.1f};
    };

    ComplementaryFilter();
    explicit ComplementaryFilter(const Config& cfg);

    // ─── API principal ────────────────────────────────────────────────────────

    // Actualiza el filtro con una muestra de IMU.
    // gyro [rad/s], accel [m/s^2], en frame del cuerpo.
    // Llamar a alta frecuencia (~200 Hz).
    void processImu(double timestamp_s, const Vec3& gyro, const Vec3& accel);

    // Aplica una corrección de yaw usando el resultado de VO estéreo.
    // dR_body: rotación relativa del cuerpo entre q_prev y q_now (frame cuerpo).
    // q_at_capture: orientación al momento de capturar el frame anterior.
    // confidence: calidad del resultado de VO en [0,1].
    void applyVisualUpdate(const Quaternion& dR_body,
                           const Quaternion& q_at_capture,
                           float confidence);

    // ─── Consultas ────────────────────────────────────────────────────────────

    // Orientación actual: body --> world (Y-down).
    Quaternion orientation() const noexcept { return q_; }

    // Bias integral acumulado del error de acelerómetro [rad/s].
    Vec3 integralBias() const noexcept { return integral_bias_; }

    // Velocidad angular filtrada (debiasada) en frame del cuerpo [rad/s].
    Vec3 angularVelocity() const noexcept { return last_omega_; }

    double lastTimestamp() const noexcept { return last_t_; }

    // ─── Control ──────────────────────────────────────────────────────────────

    void resetOrientation(const Quaternion& q = Quaternion::identity());
    void reset();

    const Config& config() const noexcept { return cfg_; }

private:
    Config     cfg_;
    Quaternion q_{Quaternion::identity()};
    Vec3       integral_bias_{};    // integral error [rad/s] (Mahony)
    Vec3       last_omega_{};
    double     last_t_{-1.0};
    Quaternion q_at_last_visual_{Quaternion::identity()};
    bool       has_visual_ref_{false};
};

}  // namespace nav_odometry
