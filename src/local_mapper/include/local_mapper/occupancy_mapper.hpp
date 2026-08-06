#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// OccupancyMapper
//
// Mapa de ocupación 2D local deslizante en log-odds (filtro bayesiano).
//
// La rejilla vive en el frame odom (absoluto, plano XZ). Al insertar
// obstáculos en coordenadas odom, el grid no necesita rotar cuando la cámara
// gira en yaw: los obstáculos quedan fijos en el mundo y el mapa es correcto
// independientemente de la orientación de la cámara.
//
// El método shift() desplaza el contenido de la ventana cuando la cámara
// se aleja del centro del grid, y actualiza origin_x_/origin_z_ para que
// worldToCell() siga siendo correcto. Las celdas que entran por el borde se
// inicializan al prior (0).
//
// Los puntos de obstáculo se reciben en coordenadas XZ del frame odom.
// El sensor se pasa también en coordenadas odom. Las celdas libres se marcan
// mediante ray casting con el algoritmo de Bresenham (Elfes, 1989).
//
// Publicación: nav_msgs/OccupancyGrid en el frame "odom".
//   Celda sin observar: -1 (unknown).
//   Celda libre:         0-49  (log-odds negativo).
//   Celda ocupada:      51-100 (log-odds positivo).
//
// Sin dependencias externas: C++17 puro.
// ─────────────────────────────────────────────────────────────────────────────

#include <vector>
#include <cstdint>
#include <cmath>
#include <functional>

namespace local_mapper {

class OccupancyMapper {
 public:
  // ── Tipos públicos ──────────────────────────────────────────────────────────

  /// Punto de obstáculo proyectado al plano XZ del marco odom.
  struct Point2D {
    float x;  ///< Coordenada X en el marco odom (m)
    float z;  ///< Coordenada Z en el marco odom (m)
  };

  // ── Configuración ───────────────────────────────────────────────────────────

  struct Config {
    /// Tamaño de celda cuadrada (m). Define la resolución lateral del mapa.
    float cell_size_m  = 0.10f;

    /// Número de celdas por lado (el mapa cubre cell_size_m × grid_size metros
    /// en X y en Z). La ventana se mantiene centrada en la posición del sensor.
    int   grid_size    = 100;

    /// Incremento de log-odds por observación ocupada.
    /// Corresponde a log P(occ|z=hit) / P(free|z=hit).
    /// Valor inicial de diseño: log(0.9/0.1) ≈ 2.197.
    /// La probabilidad 0.9 es configurable y debe calibrarse experimentalmente.
    float l_occ = 2.197f;

    /// Incremento de log-odds por observación libre (celda en el rayo).
    /// Valor inicial de diseño: log P(occ|z=free) / P(free|z=free)
    /// = log(0.3/0.7) < 0. La probabilidad 0.3 es configurable y debe
    /// calibrarse experimentalmente.
    /// El valor absoluto es menor que l_occ para que la ocupación se acumule
    /// más rápido que se borra: se necesitan ~2.6 incrementos libres para
    /// compensar un incremento ocupado e invertir el signo del log-odds.
    float l_free = -0.847f;

    /// Límite inferior de acumulación (log-odds): P ≈ 0.67 %.
    float l_min = -5.0f;

    /// Límite superior de acumulación (log-odds): P ≈ 99.3 %.
    float l_max =  5.0f;

    /// Distancia máxima al obstáculo (m). Los puntos más lejanos se descartan.
    float max_range_m = 5.0f;

    /// Radio de localidad (m). En cada llamada a update(), todas las celdas
    /// cuya distancia al sensor supera este valor se reinician al prior
    /// uniforme (log-odds = 0). Mantiene el mapa estrictamente local.
    /// Poner en 0 para deshabilitar.
    float forget_radius_m = 5.0f;

    /// Activa el ray casting de espacio libre entre el sensor y cada obstáculo.
    /// Desactivar solo para depuración (sin ray casting el mapa acumula
    /// ocupación sin nunca decrementar las celdas libres).
    bool  enable_raycasting = true;
  };

  // ── Constructor ─────────────────────────────────────────────────────────────

  OccupancyMapper() : OccupancyMapper(Config{}) {}
  explicit OccupancyMapper(const Config& cfg);

  // ── Pipeline principal ──────────────────────────────────────────────────────

  /**
   * Actualiza la rejilla con los obstáculos y los rayos libres del fotograma.
   *
   * @param obstacle_pts        Puntos de obstáculo en el marco odom (plano XZ).
   * @param sensor_x            Posición X del sensor en el marco odom (m).
   * @param sensor_z            Posición Z del sensor en el marco odom (m).
   * @param free_ray_endpoints  Extremos de rayos que no terminan en obstáculo
   *                            (p. ej. puntos más allá del rango, suelo, techo).
   *                            Se castea un rayo libre hasta cada endpoint,
   *                            incluyendo la celda final (sin marcarla ocupada).
   *                            Submuestre externamente para controlar la CPU.
   */
  void update(const std::vector<Point2D>& obstacle_pts,
              float sensor_x, float sensor_z,
              const std::vector<Point2D>& free_ray_endpoints = {});

  /**
   * Desplaza el contenido de la rejilla (shift_ci, shift_cj) celdas y
   * actualiza origin_x_/origin_z_ en la misma cantidad.
   * Las celdas nuevas que entran por el borde se inicializan a 0 (prior).
   * Se llama antes de update() con el desplazamiento de la cámara desde
   * el frame anterior, expresado en coordenadas odom y convertido a celdas.
   *
   * Convenio de signo:
   *   shift_ci > 0 → contenido se mueve hacia +X (cámara movió hacia −X en odom).
   *   shift_cj > 0 → contenido se mueve hacia +Z (cámara movió hacia −Z en odom).
   */
  void shift(int shift_ci, int shift_cj);

  // ── Acceso a resultados ─────────────────────────────────────────────────────

  /// Log-odds de cada celda. Tamaño: grid_size × grid_size.
  /// Índice: row * grid_size + col, donde row ↔ X, col ↔ Z.
  const std::vector<float>& logOdds() const { return grid_; }

  /// Número de celdas por lado.
  int   gridSize()   const { return cfg_.grid_size; }

  /// Tamaño de celda en metros.
  float cellSizeM()  const { return cfg_.cell_size_m; }

  /// Coordenada X del vértice inferior-izquierdo de la rejilla (m, marco odom).
  float originX()    const { return origin_x_; }

  /// Coordenada Z del vértice inferior-izquierdo de la rejilla (m, marco odom).
  float originZ()    const { return origin_z_; }

  /**
   * Convierte el log-odds de un valor log-odds l a probabilidad de ocupación.
   *
   * P(occ) = 1 / (1 + exp(-l))
   */
  static float logOddsToProb(float l) noexcept {
    return 1.0f / (1.0f + std::exp(-l));
  }

  /**
   * Convierte a valor nav_msgs/OccupancyGrid [0, 100] o -1 (desconocido).
   * Una celda se considera "nunca observada" si su log-odds permanece en el
   * valor inicial exacto (0.0f). En ese caso se emite -1.
   */
  static int8_t logOddsToNavMsg(float l) noexcept {
    constexpr float kUnknownThreshold = 1e-4f;
    if (std::fabs(l) < kUnknownThreshold) return -1;  // sin observación
    const float p = logOddsToProb(l);
    const int val = static_cast<int>(p * 100.0f + 0.5f);
    if (val < 0)   return 0;
    if (val > 100) return 100;
    return static_cast<int8_t>(val);
  }

 private:
  Config              cfg_;
  std::vector<float>  grid_;   ///< Log-odds, indexado [row*grid_size + col]
  float               origin_x_;
  float               origin_z_;

  /// Convierte coordenadas del mundo (x,z) a índices de celda (ci, cj).
  /// Retorna true si la celda cae dentro de la rejilla.
  bool worldToCell(float x, float z, int& ci, int& cj) const noexcept;

  /// Ray casting de Bresenham desde (x0,z0) hasta (x1,z1) (excluye el
  /// extremo final). Llama visitor(ci, cj) por cada celda intermedia.
  void castRay(int x0, int z0, int x1, int z1,
               const std::function<void(int, int)>& visitor) const;

  /// Incrementa el log-odds de la celda (ci, cj) en delta y clampea.
  void updateCell(int ci, int cj, float delta) noexcept;
};

}  // namespace local_mapper
