// ─────────────────────────────────────────────────────────────────────────────
// nav_odometry/complementary_filter.cpp
//
// Filtro complementario de Mahony para fusión IMU + visión estéreo.
//
// Algoritmo (Mahony 2008, IEEE Trans. Automatic Control):
//
//   1. Error de inclinación desde acelerómetro:
//        v  = q^{-1} * g_world   (dirección esperada de gravedad en body)
//        e  = a_norm x v          (eje de corrección en body frame)
//
//   2. Integral del error (eliminación de offset DC):
//        ib += e * Ki * dt
//
//   3. Corrección del giroscopio:
//        omega_corr = omega + e*Kp + ib
//
//   4. Integración del cuaternión:
//        q = q * exp(0.5*omega_corr*dt)
//
//   5. Corrección de yaw desde VO estéreo (cuando disponible):
//        - Extraer componente yaw del error entre rotación visual y rotación IMU
//        - Aplicar fracción alpha_yaw de la corrección al cuaternión actual
//
// Convención interna: frame del mundo Y-abajo. Referencia del acelerómetro: {0, -g, 0}
// (el sensor mide fuerza específica = -gravedad; con Y-abajo, en reposo mide -g en Y).
// ─────────────────────────────────────────────────────────────────────────────

#include "nav_odometry/complementary_filter.hpp"
#include <cmath>

namespace nav_odometry {

ComplementaryFilter::ComplementaryFilter()
    : ComplementaryFilter(Config{}) {}

ComplementaryFilter::ComplementaryFilter(const Config& cfg)
    : cfg_(cfg) {}

void ComplementaryFilter::processImu(double timestamp_s,
                                     const Vec3& gyro,
                                     const Vec3& accel)
{
    if (last_t_ < 0.0) {
        // primer procesamiento, no hay last_t_ (= -1)
        last_t_ = timestamp_s;
        last_omega_ = gyro;
        return;
    }

    const float dt = static_cast<float>(timestamp_s - last_t_);
    last_t_ = timestamp_s;

    if (dt <= 0.f || dt > cfg_.max_dt_s) {
        // dt inválido o salto temporal grande: ignorar muestra
        last_omega_ = gyro;
        return;
    }

    Vec3 w = gyro;  // velocidad angular en frame del cuerpo

    // ── Corrección de tilt (pitch/roll) desde acelerómetro ────────────────────
    const float a_norm = accel.norm();
    const float g_expected = cfg_.gravity_magnitude;
    const float g_lo = g_expected * (1.f - cfg_.gravity_mag_tolerance);
    const float g_hi = g_expected * (1.f + cfg_.gravity_mag_tolerance);

    if (a_norm > g_lo && a_norm < g_hi) {
        // Vector de gravedad normalizado medido en frame del cuerpo
        const Vec3 a_hat{accel.x / a_norm, accel.y / a_norm, accel.z / a_norm};

        // Dirección esperada de la fuerza específica en frame del cuerpo:
        //   v = q^{-1} * ref,  ref = {0, -1, 0}
        // El acelerómetro mide fuerza específica = -gravedad.
        // Con mundo Y-abajo: gravedad = {0,+1,0} --> sensor mide {0,-1,0} en reposo.
        // Para que la corrección sea nula en la orientación correcta (identidad),
        // la referencia debe ser la fuerza específica esperada, no la gravedad.
        const Vec3 g_world_norm{0.f, -1.f, 0.f};
        const Vec3 v = q_.conjugate().rotate(g_world_norm);

        // Error de inclinación: eje alrededor del cual rotar v para alinearlo con a_hat.
        // a_hat x v da el eje en frame del cuerpo (sigue la regla de la mano derecha).
        const Vec3 e = a_hat.cross(v);

        // Actualizar integral de error (elimina sesgo DC persistente)
        integral_bias_ += e * (cfg_.ki_accel * dt);

        // Corrección proporcional + integral sobre la velocidad angular
        w.x += e.x * cfg_.kp_accel + integral_bias_.x;
        w.y += e.y * cfg_.kp_accel + integral_bias_.y;
        w.z += e.z * cfg_.kp_accel + integral_bias_.z;
    }

    // ── Integración del cuaternión ────────────────────────────────────────────
    const Vec3 omega_dt{w.x * dt, w.y * dt, w.z * dt};
    q_ = (q_ * Quaternion::fromOmegaDt(omega_dt)).normalized();
    last_omega_ = w;
}

void ComplementaryFilter::applyVisualUpdate(const Quaternion& dR_body,
                                             const Quaternion& q_at_capture,
                                             float confidence)
{
    if (confidence < cfg_.min_visual_confidence) return;

    // Orientación que el filtro habría producido si la rotación visual fuera exacta:
    //   q_visual = q_at_capture * dR_body
    const Quaternion q_visual = (q_at_capture * dR_body).normalized();

    // Calcular la diferencia entre la estimación actual del filtro y la visual.
    // dR_error = q_.conjugate() * q_visual
    //          representa la rotación en body frame del error de la estimación IMU.
    const Quaternion dR_error = (q_.conjugate() * q_visual).normalized();

    // Extraer el componente de yaw del error (rotación alrededor del eje Y del mundo).
    // Para ello convertir dR_error al frame del mundo y aislar la componente Y.
    //   dR_error_world = q_ * dR_error * q_.conjugate()   (transformación de adjunta)
    const Quaternion dR_error_world = (q_ * dR_error * q_.conjugate()).normalized();

    // El ángulo de yaw del error en el frame del mundo es el que corregimos.
    // Para ángulos pequeños: yaw_error ~= 2 * dR_error_world.y  (eje Y)
    const float yaw_error = 2.f * dR_error_world.y;  // radianes, pequeño ángulo

    // Ganancia de corrección: alpha_yaw ponderada por la confianza de la VO
    const float gain = cfg_.alpha_yaw * confidence;

    // Corrección de yaw aplicada en el frame del mundo (left-multiply):
    //   q_corrected = dq_yaw_world * q_
    // donde dq_yaw_world es una rotación pura alrededor del eje Y del mundo.
    const Quaternion dq_yaw = Quaternion::fromAngleAxis(
        yaw_error * gain, Vec3{0.f, 1.f, 0.f});
    q_ = (dq_yaw * q_).normalized();

    // También aplicar la corrección de roll/pitch implícita en la diferencia visual
    // (con ganancia reducida para no interferir con el Mahony de acelerómetro).
    // Se usa el error completo con ganancia = alpha_yaw * 0.1 (corrección débil).
    const float roll_pitch_gain = cfg_.alpha_yaw * 0.1f * confidence;
    if (roll_pitch_gain > 0.f) {
        // Aplicar slerp hacia q_visual con ganancia muy pequeña
        q_ = Quaternion::slerp(q_, q_visual, roll_pitch_gain);
        q_ = q_.normalized();
    }

    has_visual_ref_ = true;
    q_at_last_visual_ = q_;
}

void ComplementaryFilter::resetOrientation(const Quaternion& q) {
    q_             = q.normalized();
    integral_bias_ = {};
    has_visual_ref_ = false;
    q_at_last_visual_ = q_;
}

void ComplementaryFilter::reset() {
    q_             = Quaternion::identity();
    integral_bias_ = {};
    last_omega_    = {};
    last_t_        = -1.0;
    has_visual_ref_ = false;
    q_at_last_visual_ = Quaternion::identity();
}

}  // namespace nav_odometry
