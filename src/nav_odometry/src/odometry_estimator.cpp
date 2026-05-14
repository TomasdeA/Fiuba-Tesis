// ─────────────────────────────────────────────────────────────────────────────
// nav_odometry/odometry_estimator.cpp
//
// Orquesta ComplementaryFilter + RgbdTracker para producir odometría VIO.
//
// Flujo de datos:
//   processImu  ---> ComplementaryFilter (alta frecuencia, ~200 Hz gyro / ~100 Hz accel)
//
//   processRgbd  --> RgbdTracker (baja frecuencia, ~30 fps)
//                 |-> ComplementaryFilter.applyVisualUpdate (corrección yaw)
//                 |-> position_ += dT (integración de translación visual)
// ─────────────────────────────────────────────────────────────────────────────

#include "nav_odometry/odometry_estimator.hpp"
#include <chrono>
#include <cmath>

namespace {
// Helper: tiempo transcurrido en microsegundos desde t0.
inline double us_since(const std::chrono::steady_clock::time_point& t0) {
    return std::chrono::duration<double, std::micro>(
        std::chrono::steady_clock::now() - t0).count();
}
} // namespace

namespace nav_odometry {

OdometryEstimator::OdometryEstimator()
    : OdometryEstimator(Config{}) {}

OdometryEstimator::OdometryEstimator(const Config& cfg)
    : cfg_(cfg)
    , filter_(cfg.filter)
    , tracker_(cfg.tracker)
{}

void OdometryEstimator::setCameraIntrinsics(const CameraIntrinsics& intrinsics) {
    tracker_.setCameraIntrinsics(intrinsics);
}

void OdometryEstimator::setDepthFrame(const DepthFrame& frame) {
    tracker_.setDepthFrame(frame);
}

void OdometryEstimator::processImu(const ImuSample& sample) {
    {
        const auto t0 = std::chrono::steady_clock::now();
        filter_.processImu(sample.timestamp_s, sample.gyro, sample.accel);
        perf_.imu_filter.record(us_since(t0));
    }

    // Actualizar caché de estado a alta frecuencia
    std::lock_guard<std::mutex> lock(state_mutex_);
    cached_state_.timestamp_s      = sample.timestamp_s;
    cached_state_.pose.orientation = filter_.orientation();
    cached_state_.pose.position    = position_;
    cached_state_.angular_velocity = filter_.angularVelocity();
    cached_state_.valid            = true;
}

void OdometryEstimator::processRgbd(const ColorFrame& frame)
{
    const auto t_stereo_start = std::chrono::steady_clock::now();

    // Capturar orientación justo antes de procesar el frame visual
    const Quaternion q_at_capture = filter_.orientation();

    // Ejecutar VO
    VisualOdometryResult vo;
    {
        const auto t0 = std::chrono::steady_clock::now();
        vo = tracker_.process(frame);
        perf_.tracker.record(us_since(t0));
    }

    if (!vo.valid) {
        last_visual_t_     = frame.timestamp_s;
        q_at_last_visual_  = q_at_capture;
        return;
    }

    // Verificación de velocidad plausible: |dT|/dt < max_velocity
    if (last_visual_t_ > 0.0) {
        const float dt_vis = static_cast<float>(frame.timestamp_s - last_visual_t_);
        const float t_mag  = vo.delta_translation.norm();
        if (dt_vis > 0.f && t_mag / dt_vis > cfg_.max_velocity_mps) {
            last_visual_t_    = frame.timestamp_s;
            q_at_last_visual_ = q_at_capture;
            return;  // estimación de velocidad no plausible
        }
    }

    // ── Aplicar corrección de yaw al filtro + integrar posición ─────────────
    {
        const auto t0 = std::chrono::steady_clock::now();

        // Capturar la orientación en t-1 ANTES de aplicar la corrección visual,
        // ya que delta_translation está expresado en el frame de la cámara en t-1.
        const Quaternion q_t_minus_1 = q_at_last_visual_;

        filter_.applyVisualUpdate(vo.delta_rotation, q_at_last_visual_, vo.confidence);

        // La translación del tracker está en frame de la cámara en t-1.
        // Se convierte al frame del mundo usando la orientación en t-1 (q_t_minus_1),
        // NO la orientación post-update, que introduce error cuando hay corrección de yaw.
        const Vec3 dT_world = q_t_minus_1.rotate(vo.delta_translation);

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            position_ += dT_world * cfg_.translation_confidence_scale;

            // Velocidad lineal aproximada (diferencia de posición / dt)
            if (last_visual_t_ > 0.0) {
                const float dt_vis = static_cast<float>(frame.timestamp_s - last_visual_t_);
                if (dt_vis > 0.f) {
                    velocity_.x = dT_world.x / dt_vis;
                    velocity_.y = dT_world.y / dt_vis;
                    velocity_.z = dT_world.z / dt_vis;
                }
            }

            cached_state_.timestamp_s      = frame.timestamp_s;
            cached_state_.pose.orientation  = filter_.orientation();
            cached_state_.pose.position     = position_;
            cached_state_.linear_velocity   = velocity_;
            cached_state_.angular_velocity  = filter_.angularVelocity();
            cached_state_.valid             = true;
        }

        perf_.visual_update.record(us_since(t0));
    }

    perf_.rgbd_total.record(us_since(t_stereo_start));

    last_visual_t_    = frame.timestamp_s;
    q_at_last_visual_ = filter_.orientation();
}

OdometryState OdometryEstimator::getState() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return cached_state_;
}

void OdometryEstimator::reset() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    filter_.reset();
    tracker_.reset();
    position_         = {};
    velocity_         = {};
    last_visual_t_    = -1.0;
    q_at_last_visual_ = Quaternion::identity();
    cached_state_     = OdometryState{};
    perf_.reset();
}

}  // namespace nav_odometry
