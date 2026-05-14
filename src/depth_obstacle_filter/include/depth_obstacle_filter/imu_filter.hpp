#pragma once

#include <rclcpp/rclcpp.hpp>

namespace depth_obstacle_filter {

/**
 * Filtrado de señales del IMU para estimación de orientación estática.
 *
 * Responsabilidades:
 *  - Rechazo de outliers del acelerómetro (golpes, tropiezos).
 *  - Filtro IIR low-pass de primer orden sobre el acelerómetro.
 *  - Estimación y substracción del bias del giroscopio.
 *  - Acumulación del heading (guiñada) a partir del giroscopio.
 *
 * No depende de GravityAligner: este módulo solo provee aceleración filtrada.
 * GravityAligner se encarga de convertir esa aceleración en una rotación.
 */
class ImuFilter {
 public:
  struct Config {
    bool  use_gyro      = false;
    float gyro_deadzone = 0.008f;  ///< [rad/s] filtra ruido del giroscopio
  };

  ImuFilter(const Config& cfg, rclcpp::Logger logger,
            rclcpp::Clock::SharedPtr clock);

  // ── API pública ─────────────────────────────────────────────────────────────

  /**
   * Procesa una muestra del acelerómetro.
   * Aplica rechazo de outliers y filtro IIR low-pass.
   * @param ax, ay, az  Aceleración en m/s² (frame óptico del sensor)
   */
  void processAccel(float ax, float ay, float az);

  /**
   * Procesa una muestra del giroscopio.
   * Acumula bias durante la calibración; integra heading si use_gyro=true.
   * @param wy          Velocidad angular en el eje Y (rad/s)
   * @param stamp       Timestamp de la muestra
   * @param calibrating true mientras se está en fase de calibración inicial
   */
  void processGyro(float wy, const rclcpp::Time& stamp, bool calibrating);

  /** Cierra la fase de calibración: fija el bias promedio y resetea el heading. */
  void finishCalibration();

  // ── Getters ─────────────────────────────────────────────────────────────────

  float ax() const { return ax_filt_; }
  float ay() const { return ay_filt_; }
  float az() const { return az_filt_; }

  float heading() const { return heading_rad_; }

  /** true cuando ‖a_filtrado‖ se aleja más de 0.5 m/s² de 1 g (movimiento activo). */
  bool isDynamic() const { return accel_dynamic_; }

 private:
  Config cfg_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;

  // Estado del filtro IIR del acelerómetro
  // Inicializado en (0, -g, 0): cámara horizontal, sin corrección requerida.
  // Los outliers se descartan sin modificar este estado; el IIR converge
  // hacia el valor real a partir de la primera muestra válida.
  float ax_filt_ = 0.0f;
  float ay_filt_ = -9.81f;
  float az_filt_ = 0.0f;
  bool  accel_dynamic_ = false;

  // Estado del giroscopio
  float heading_rad_     = 0.0f;
  float gyro_bias_       = 0.0f;
  float gyro_bias_accum_ = 0.0f;
  int   gyro_bias_count_ = 0;
  rclcpp::Time last_gyro_stamp_{0, 0, RCL_ROS_TIME};
};

}  // namespace depth_obstacle_filter
