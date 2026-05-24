#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// nav_odometry/odometry_estimator.hpp
//
// Backend concreto de odometría VIO que implementa IOdometryBackend.
// Compone:
//   - ComplementaryFilter (Mahony): fusión IMU + correcciones de visión.
//   - RgbdTracker (LK + PnP): odometría visual con profundidad de hardware.
//
// Estrategia de fusión:
//   1. IMU a alta frecuencia --> actualiza orientación y velocidad angular.
//   2. Cada frame visual --> estima dR y dT visuales (LK + PnP).
//   3. dR visual se aplica al filtro como corrección de yaw.
//   4. dT visual acumula la posición (no se integra acelerómetro).
// ─────────────────────────────────────────────────────────────────────────────

#include "nav_odometry/iodometry_backend.hpp"
#include "nav_odometry/complementary_filter.hpp"
#include "nav_odometry/rgbd_tracker.hpp"

#include <chrono>
#include <limits>
#include <mutex>

namespace nav_odometry {

// ─── Estadísticas de tiempo por sección ──────────────────────────────────────
struct SectionStats {
    int64_t count{0};
    double  total_us{0.0};
    double  min_us{std::numeric_limits<double>::max()};
    double  max_us{0.0};

    double avg_us()     const noexcept { return count > 0 ? total_us / static_cast<double>(count) : 0.0; }
    double min_display() const noexcept { return count > 0 ? min_us : 0.0; }
    double max_display() const noexcept { return count > 0 ? max_us : 0.0; }
    void record(double elapsed_us) noexcept {
        ++count;
        total_us += elapsed_us;
        if (elapsed_us < min_us) min_us = elapsed_us;
        if (elapsed_us > max_us) max_us = elapsed_us;
    }
    void reset() noexcept { *this = SectionStats{}; }
};

// ─── Reporte de rendimiento del estimador ────────────────────────────────────
struct PerfReport {
    SectionStats imu_filter;       // ComplementaryFilter::processImu
    SectionStats tracker;          // RgbdTracker::process
    SectionStats visual_update;    // applyVisualUpdate + integración de posición
    SectionStats rgbd_total;     // processRgbd completo

    void reset() noexcept {
        imu_filter.reset();
        tracker.reset();
        visual_update.reset();
        rgbd_total.reset();
    }
};

class OdometryEstimator : public IOdometryBackend {
public:
    struct Config {
        ComplementaryFilter::Config filter{};
        RgbdTracker::Config         tracker{};
        bool collect_perf_stats{true};
        // Escala de confianza para la translación visual.
        // Si la VO es ruidosa, reducir este valor para suavizar la posición.
        float translation_confidence_scale{1.0f};
        // Velocidad máxima plausible [m/s]. Estimaciones que implican mayor
        // velocidad se descartan.
        float max_velocity_mps{3.0f};
    };

    OdometryEstimator();
    explicit OdometryEstimator(const Config& cfg);

    // ─── IOdometryBackend ─────────────────────────────────────────────────────
    void processImu(const ImuSample& sample) override;
    void processRgbd(const ColorFrame& frame) override;
    OdometryState getState() const override;
    void reset() override;
    void setCameraIntrinsics(const CameraIntrinsics& intrinsics) override;
    const char* name() const noexcept override { return "ComplementaryVIO"; }

    // Actualiza el depth image para el siguiente processRgbd().
    // Debe llamarse antes de processRgbd(). No copia datos.
    void setDepthFrame(const DepthFrame& frame);

    // ─── Temporización ────────────────────────────────────────────────────────
    // Devuelve una copia del reporte acumulado desde la última llamada a
    // resetPerfReport(). Acceso best-effort (sin lock) – apto para profiling.
    PerfReport getPerfReport() const noexcept { return perf_; }
    void       resetPerfReport() noexcept { perf_.reset(); }

private:
    Config                  cfg_;
    ComplementaryFilter     filter_;
    RgbdTracker             tracker_;

    // Pose acumulada (posición integrada desde VO, orientación del filtro)
    Vec3   position_{};
    Vec3   velocity_{};

    // Timestamp del último frame visual procesado
    double last_visual_t_{-1.0};
    Quaternion q_at_last_visual_{Quaternion::identity()};

    // Protege el estado leído desde getState()
    mutable std::mutex state_mutex_;

    // Copia local del estado para acceso sin lock en el hilo de publicación
    OdometryState cached_state_{};

    // Estadísticas de rendimiento (best-effort, sin lock extra)
    PerfReport perf_{};
};

}  // namespace nav_odometry
