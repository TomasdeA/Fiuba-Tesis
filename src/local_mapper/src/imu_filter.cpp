#include "local_mapper/imu_filter.hpp"
#include <cmath>

namespace local_mapper {

// Constantes del filtro IIR de primer orden
// Frecuencia de corte ≈ 5 Hz, frecuencia de muestreo del IMU ≈ 200 Hz
//   α = 2π·fc / (2π·fc + fs) = 31.4 / 231.4 ≈ 0.136
static constexpr float ALPHA_LP      = 0.136f;
static constexpr float G             = 9.81f;
static constexpr float OUTLIER_NORM  = 2.5f * G;  ///< 24.5 m/s² — golpe fuerte
static constexpr float DYNAMIC_THRESH = 0.5f;     ///< m/s² — movimiento activo

ImuFilter::ImuFilter(const Config& cfg, rclcpp::Logger logger,
                     rclcpp::Clock::SharedPtr clock)
    : cfg_(cfg), logger_(logger), clock_(std::move(clock)) {}

// ── Acelerómetro ─────────────────────────────────────────────────────────────

void ImuFilter::processAccel(float ax_raw, float ay_raw, float az_raw) {
  // Capa 1: rechazo de outliers por norma excesiva
  const float norm_raw = std::sqrt(ax_raw*ax_raw + ay_raw*ay_raw + az_raw*az_raw);
  if (norm_raw > OUTLIER_NORM) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 500,
        "ImuFilter: aceleración anómala (‖a‖=%.1f m/s² > %.1f) → muestra descartada",
        norm_raw, OUTLIER_NORM);
    return;  // conservar el último valor filtrado válido
  }

  // Capa 2: filtro IIR low-pass de primer orden
  ax_filt_ = ALPHA_LP * ax_raw + (1.0f - ALPHA_LP) * ax_filt_;
  ay_filt_ = ALPHA_LP * ay_raw + (1.0f - ALPHA_LP) * ay_filt_;
  az_filt_ = ALPHA_LP * az_raw + (1.0f - ALPHA_LP) * az_filt_;

  // Capa 3: detección de dinámica activa
  const float norm_filt = std::sqrt(ax_filt_*ax_filt_ + ay_filt_*ay_filt_ + az_filt_*az_filt_);
  accel_dynamic_ = (std::abs(norm_filt - G) > DYNAMIC_THRESH);
}

// ── Giroscopio ───────────────────────────────────────────────────────────────

void ImuFilter::processGyro(float wy, const rclcpp::Time& stamp, bool calibrating) {
  if (last_gyro_stamp_.nanoseconds() > 0) {
    const double dt = (stamp - last_gyro_stamp_).seconds();
    if (dt > 0.0 && dt < 0.5) {
      if (calibrating) {
        gyro_bias_accum_ += wy;
        gyro_bias_count_++;
      }
      if (cfg_.use_gyro) {
        const float wy_corr = wy - gyro_bias_;
        if (std::abs(wy_corr) > cfg_.gyro_deadzone) {
          heading_rad_ += wy_corr * static_cast<float>(dt);
        }
      }
    }
  }
  last_gyro_stamp_ = stamp;
}

void ImuFilter::finishCalibration() {
  if (gyro_bias_count_ > 0) {
    gyro_bias_ = gyro_bias_accum_ / static_cast<float>(gyro_bias_count_);
  }
  heading_rad_ = 0.0f;
}

}  // namespace local_mapper
