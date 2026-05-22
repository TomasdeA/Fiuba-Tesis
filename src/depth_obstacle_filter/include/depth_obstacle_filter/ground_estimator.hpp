#pragma once

#include <vector>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <time.h>

namespace depth_obstacle_filter {

/**
 * Estimación del plano del suelo por RANSAC jerárquico sobre la nube alineada.
 *
 * Implementa el pipeline de 8 etapas de Poux (2024):
 *
 *  Stage 1  – Diagnóstico: estadísticas de la nube (densidad, ruido, outliers)
 *  Stage 2  – Voxel sampling: reduce densidad no uniforme a resolución fija
 *  Stage 3  – Análisis multi-escala: determina parámetros óptimos de RANSAC
 *             a partir del ruido estimado en Stage 1
 *  Stage 4  – RANSAC jerárquico: detección del plano dominante
 *  Stage 5  – Validación geométrica y métricas de calidad
 *  Stage 6  – Refinamiento por mínimos cuadrados sobre inliers finales
 *  Stage 7  – Clasificación: GROUND / OBSTACLE / CEILING
 *  Stage 8  – Análisis de performance (tiempo, n_inliers, calidad)
 *
 * Sistema de coordenadas de entrada: nube ya alineada a gravedad.
 *   Y positivo = abajo. El plano del suelo es el más bajo (Y máximo).
 */
class GroundEstimator {
 public:
  // ── Tipos públicos ──────────────────────────────────────────────────────────

  struct Point3D {
    float x, y, z;
  };

  /// Plano representado por su normal y distancia al origen: n·p + d = 0
  struct Plane {
    float nx, ny, nz;  ///< Normal unitaria (apunta "arriba" en frame alineado)
    float d;           ///< Distancia al origen
    float quality;     ///< Fracción de inliers / total [0, 1]
    int   n_inliers;   ///< Número de inliers
    bool  valid = false;
  };

  /// Estadísticas de la nube (Stage 1)
  struct CloudStats {
    float mean_nn_dist;   ///< Distancia media al vecino más cercano (m)
    float std_nn_dist;    ///< Desviación estándar de la distancia NN (m)
    float noise_estimate; ///< Estimación del ruido del sensor (m)
    float density;        ///< Puntos por m³
    int   n_outliers;     ///< Puntos a más de 3σ de la distancia NN
    int   n_total;        ///< Total de puntos de entrada
    bool  valid = false;
  };

  /// Etiqueta de cada punto tras la clasificación (Stage 7)
  enum class Label : uint8_t {
    UNKNOWN  = 0,
    GROUND   = 1,  ///< Sobre el plano del suelo detectado
    OBSTACLE = 2,  ///< Obstáculo chocable (pared, objeto, cuerpo, etc.)
    CEILING  = 3,  ///< Por encima de la cámara + ceiling_delta_m → descartable
  };

  /// Métricas de performance (Stage 8)
  struct PerfStats {
    float time_total_ms;
    float time_diag_ms;    ///< Stage 1: diagnóstico (búsqueda NN)
    float time_voxel_ms;
    float time_ransac_ms;
    float time_refine_ms;
    int   n_input;
    int   n_voxel;
    int   n_inliers;
    int   ransac_iterations;
  };

  // ── Configuración ───────────────────────────────────────────────────────────

  struct Config {
    // Stage 2 — Voxel sampling
    bool  enable_voxel_filter = true;  ///< Si false, RANSAC usa la nube sin downsampling.
    float voxel_size_m    = 0.05f;   ///< Tamaño de celda del voxel grid (m)

    // Stage 3 / 4 — RANSAC
    int   ransac_max_iter    = 100;  ///< Iteraciones máximas de RANSAC
    float ransac_inlier_tol  = 0.0f; ///< 0 → se calcula automáticamente en Stage 3
    float ransac_min_inliers = 0.3f; ///< Fracción mínima de inliers sobre la banda
    float min_person_height_m = 1.3f; ///< Altura mínima esperada de cualquier usuario (m).
                                      ///<   La cámara se monta en la cabeza, por lo que
                                      ///<   h_cam ≈ altura_persona + delta_soporte.
                                      ///<   El suelo siempre está a Y >= min_person_height_m.
                                      ///<   Solo se usan como candidatos a suelo los puntos
                                      ///<   con Y >= este valor.

    // Stage 4 — Histograma Y / tolerancia variable
    float hist_bin_m       = 0.05f;  ///< Ancho de bin del histograma de Y (m)
    float hist_band_low_m  = 0.10f;  ///< Margen superior al pico del histograma (m)
    float hist_band_high_m = 0.20f;  ///< Margen inferior al pico del histograma (m)
    float floor_noise_k    = 0.015f; ///< Factor k para ε_i = k·‖p_i‖ (modelo D435)

    // Stage 5 — Validación
    float max_tilt_deg    = 15.0f;  ///< Inclinación máxima aceptable del plano del suelo
    float min_quality     = 0.25f;  ///< Calidad mínima (fracción de inliers)

    // Stage 7 — Clasificación
    float ceiling_delta_m = 0.10f; ///< Puntos con Y < -ceiling_delta_m se clasifican
                                   ///< como CEILING y se descartan del mapa.
                                   ///< En el frame alineado Y=0 es la cámara (en la
                                   ///< cabeza); Y negativo = por encima de la cámara.

    // Diagnóstico Stage 1
    int   nn_sample_size  = 100;    ///< Puntos muestreados para calcular NN dist
  };

  // ── Constructor ─────────────────────────────────────────────────────────────

  GroundEstimator() : GroundEstimator(Config{}) {}
  explicit GroundEstimator(const Config& cfg);

  // ── Pipeline principal ──────────────────────────────────────────────────────

  /**
   * Ejecuta el pipeline completo sobre la nube alineada a gravedad.
   *
   * @param cloud  Nube alineada (frame donde Y = abajo)
   * @return true si se detectó un plano de suelo válido
   */
  bool estimate(const std::vector<Point3D>& cloud);

  // ── Getters de resultados ───────────────────────────────────────────────────

  const Plane&      groundPlane()  const { return ground_plane_; }
  const CloudStats& cloudStats()   const { return stats_; }
  const PerfStats&  perfStats()    const { return perf_; }

  /// Etiquetas de los puntos de la última nube procesada (misma longitud)
  const std::vector<Label>& labels() const { return labels_; }

  /// Índices de los puntos clasificados como GROUND
  const std::vector<int>& groundIndices()   const { return ground_idx_; }
  /// Índices de los puntos clasificados como OBSTACLE
  const std::vector<int>& obstacleIndices() const { return obstacle_idx_; }
  /// Índices de los puntos clasificados como CEILING
  const std::vector<int>& ceilingIndices()  const { return ceiling_idx_; }

  /// Altura de la cámara sobre el suelo en metros
  float cameraHeightM() const;

  bool isValid() const { return ground_plane_.valid; }

 private:
  // ── Etapas individuales ─────────────────────────────────────────────────────

  /// Stage 1: estadísticas de la nube
  CloudStats stage1_diagnostics(const std::vector<Point3D>& cloud) const;

  /// Stage 2: voxel downsampling
  std::vector<Point3D> stage2_voxel_sample(const std::vector<Point3D>& cloud) const;

  /// Stage 3: calcular tolerancia RANSAC adaptativa desde el ruido estimado
  float stage3_adaptive_tolerance(const CloudStats& stats) const;

  /// Stage 4: RANSAC jerárquico, retorna plano candidato
  Plane stage4_ransac(const std::vector<Point3D>& cloud, float tol, int max_iter) const;

  /// Stage 5: validación geométrica del plano
  bool stage5_validate(const Plane& plane) const;

  /// Stage 6: refinar plano por mínimos cuadrados sobre inliers
  Plane stage6_refine(const std::vector<Point3D>& cloud,
                      const Plane& initial, float tol) const;

  /// Stage 7: clasificar todos los puntos según orientación del plano
  void stage7_classify(const std::vector<Point3D>& cloud,
                       const Plane& plane, float tol);

  // ── Utilidades internas ─────────────────────────────────────────────────────

  /// Distancia con signo de un punto al plano
  static float pointPlaneDist(const Point3D& p, const Plane& pl);

  /// Ajuste de plano por mínimos cuadrados sobre un conjunto de puntos
  static Plane fitPlaneLS(const std::vector<const Point3D*>& pts);

  /// Tiempo en milisegundos desde una marca de inicio
  static float elapsedMs(struct timespec start);

  // ── Estado ──────────────────────────────────────────────────────────────────
  Config              cfg_;
  Plane               ground_plane_;
  CloudStats          stats_;
  PerfStats           perf_{};
  std::vector<Label>  labels_;
  std::vector<int>    ground_idx_;
  std::vector<int>    obstacle_idx_;
  std::vector<int>    ceiling_idx_;
};

}  // namespace depth_obstacle_filter
