// ─────────────────────────────────────────────────────────────────────────────
// OccupancyMapper
//
// Mapa de ocupación 2D en log-odds con ray casting de espacio libre.
// Ver occupancy_mapper.hpp para la descripción completa del módulo.
// ─────────────────────────────────────────────────────────────────────────────

#include "local_mapper/occupancy_mapper.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace local_mapper {

// ─────────────────────────────────────────────────────────────────────────────
// Constructor
// ─────────────────────────────────────────────────────────────────────────────

OccupancyMapper::OccupancyMapper(const Config& cfg)
    : cfg_(cfg)
{
  const int n = cfg_.grid_size * cfg_.grid_size;
  grid_.assign(n, 0.0f);  // Prior uniforme: P(occ) = 0.5 → log-odds = 0

  // La rejilla se centra en el origen del marco odom (posición inicial del nodo).
  // origin_x_ y origin_z_ corresponden al vértice (ci=0, cj=0) de la rejilla.
  const float half = static_cast<float>(cfg_.grid_size) * cfg_.cell_size_m * 0.5f;
  origin_x_ = -half;
  origin_z_ = -half;
}

// ─────────────────────────────────────────────────────────────────────────────
// worldToCell
//
// Convierte coordenadas (x, z) del marco odom a índices de celda (ci, cj).
// ci ↔ eje X, cj ↔ eje Z.
// Retorna false si el punto cae fuera de la rejilla.
// ─────────────────────────────────────────────────────────────────────────────

bool OccupancyMapper::worldToCell(float x, float z,
                                  int& ci, int& cj) const noexcept
{
  const float inv = 1.0f / cfg_.cell_size_m;
  ci = static_cast<int>(std::floor((x - origin_x_) * inv));
  cj = static_cast<int>(std::floor((z - origin_z_) * inv));
  return ci >= 0 && ci < cfg_.grid_size &&
         cj >= 0 && cj < cfg_.grid_size;
}

// ─────────────────────────────────────────────────────────────────────────────
// updateCell
// ─────────────────────────────────────────────────────────────────────────────

void OccupancyMapper::updateCell(int ci, int cj, float delta) noexcept
{
  float& cell = grid_[ci * cfg_.grid_size + cj];
  cell = std::min(cfg_.l_max, std::max(cfg_.l_min, cell + delta));
}

// ─────────────────────────────────────────────────────────────────────────────
// castRay
//
// Implementación del algoritmo de Bresenham para recorrer las celdas
// de la rejilla entre los puntos (x0,z0) y (x1,z1) en coordenadas de celda.
//
// Todas las celdas a lo largo del rayo (excepto la celda final (x1,z1),
// que corresponde al obstáculo y se actualiza por separado en update())
// reciben una observación de espacio libre (l_free).
//
// El algoritmo de Bresenham garantiza que cada celda atravesada por el
// segmento de recta recibe exactamente una llamada, sin repeticiones ni
// saltos, lo que produce un ray casting coherente con la discretización
// de la rejilla.
// ─────────────────────────────────────────────────────────────────────────────

void OccupancyMapper::castRay(int x0, int z0, int x1, int z1,
                              const std::function<void(int, int)>& visitor) const
{
  const int dx =  std::abs(x1 - x0);
  const int dz =  std::abs(z1 - z0);
  const int sx = (x0 < x1) ? 1 : -1;
  const int sz = (z0 < z1) ? 1 : -1;

  int err = dx - dz;

  // Recorre desde (x0,z0) hasta —pero sin incluir— (x1,z1).
  while (x0 != x1 || z0 != z1) {
    visitor(x0, z0);

    const int e2 = 2 * err;
    if (e2 > -dz) { err -= dz; x0 += sx; }
    if (e2 <  dx) { err += dx; z0 += sz; }
  }
  // La celda (x1,z1) NO se incluye aquí; se marca ocupada en update().
}

// ─────────────────────────────────────────────────────────────────────────────
// shift
//
// Desplaza el contenido completo de la rejilla (shift_ci, shift_cj) celdas
// y actualiza origin_x_/z_ en la misma magnitud, de modo que worldToCell()
// siga siendo correcto cuando el sensor se aleja del origen de odom.
// Cada celda (ci, cj) se mueve a (ci + shift_ci, cj + shift_cj).
// Las celdas que caen fuera de los límites se descartan.
// Las celdas nuevas (que entran por el borde opuesto) se inicializan a 0
// (prior uniforme P = 0.5), lo que las deja marcadas como "sin observar".
//
// Esta operación se llama en el nodo antes de update() con el desplazamiento
// de la cámara en coordenadas odom (no en gaf), de modo que el grid sigue
// siendo correcto en el frame odom independientemente del yaw de la cámara.
// Tiene complejidad O(N²) donde N = grid_size.
// ─────────────────────────────────────────────────────────────────────────────

void OccupancyMapper::shift(int shift_ci, int shift_cj)
{
  if (shift_ci == 0 && shift_cj == 0) return;

  const int N = cfg_.grid_size;
  std::vector<float> new_grid(static_cast<std::size_t>(N * N), 0.0f);

  for (int ci = 0; ci < N; ++ci) {
    const int new_ci = ci + shift_ci;
    if (new_ci < 0 || new_ci >= N) continue;
    for (int cj = 0; cj < N; ++cj) {
      const int new_cj = cj + shift_cj;
      if (new_cj < 0 || new_cj >= N) continue;
      new_grid[static_cast<std::size_t>(new_ci * N + new_cj)] =
          grid_[static_cast<std::size_t>(ci * N + cj)];
    }
  }
  grid_ = std::move(new_grid);

  // Actualizar el origen en el marco odom: cada celda se desplazó +shift,
  // por lo que el vértice (0,0) apunta ahora a una posición diferente.
  origin_x_ -= static_cast<float>(shift_ci) * cfg_.cell_size_m;
  origin_z_ -= static_cast<float>(shift_cj) * cfg_.cell_size_m;
}

// ─────────────────────────────────────────────────────────────────────────────
// update
//
// Actualiza la rejilla con los obstáculos del fotograma actual.
//
// Para cada punto de obstáculo en el marco gravity_aligned_frame (XZ relativo
// a la cámara, sensor siempre en (0,0)):
//
//   1. Se comprueba que el obstáculo esté dentro del rango máximo y
//      dentro de los límites de la rejilla.
//
//   2. Se actualiza la celda del obstáculo con l_occ (observación ocupada).
//
//   3. Si enable_raycasting=true, se recorren las celdas intermedias entre
//      el sensor y el obstáculo con el algoritmo de Bresenham, incrementando
//      cada una en l_free (observación libre).
//
// Referencia del modelo: Elfes (1989); Thrun et al. (2005), Sección 9.2.
// ─────────────────────────────────────────────────────────────────────────────

void OccupancyMapper::update(const std::vector<Point2D>& obstacle_pts,
                             float sensor_x, float sensor_z,
                             const std::vector<Point2D>& free_ray_endpoints)
{
  // Celda del sensor (origen del ray casting).
  int sensor_ci, sensor_cj;
  if (!worldToCell(sensor_x, sensor_z, sensor_ci, sensor_cj)) return;

  // ── Borrado fuera del radio de localidad ────────────────────────────────────
  // Pase O(N²): las celdas cuya distancia al sensor supera forget_radius_m se
  // reinician al prior uniforme (log-odds = 0). Las celdas ya en prior se
  // saltan para no gastar ciclos. Esto ocurre ANTES de insertar las nuevas
  // observaciones del fotograma.
  if (cfg_.forget_radius_m > 0.0f) {
    const float inv = 1.0f / cfg_.cell_size_m;
    const float r2  = (cfg_.forget_radius_m * inv) * (cfg_.forget_radius_m * inv);
    for (int ci = 0; ci < cfg_.grid_size; ++ci) {
      const float di  = static_cast<float>(ci - sensor_ci);
      const float di2 = di * di;
      for (int cj = 0; cj < cfg_.grid_size; ++cj) {
        auto& cell = grid_[static_cast<std::size_t>(ci * cfg_.grid_size + cj)];
        if (std::fabs(cell) < 1e-4f) continue;  // ya en prior: saltar
        if (di2 >= r2) { cell = 0.0f; continue; }  // fila entera fuera del radio
        const float dj = static_cast<float>(cj - sensor_cj);
        if (di2 + dj * dj > r2) { cell = 0.0f; }
      }
    }
  }

  const float max_range2 = cfg_.max_range_m * cfg_.max_range_m;

  // Máscara de celdas que ya recibieron l_free en este fotograma.
  // Sin esta deduplicación, N rayos que pasan por la misma celda aplicarían
  // l_free × N en un solo frame, borrando en un instante los l_occ acumulados
  // en frames anteriores. Con la máscara cada celda recibe l_free como máximo
  // una vez por llamada a update(), lo que preserva los datos históricos.
  std::vector<bool> free_updated(
      static_cast<std::size_t>(cfg_.grid_size * cfg_.grid_size), false);

  for (const auto& pt : obstacle_pts) {
    // Distancia horizontal al sensor (plano XZ).
    const float dx = pt.x - sensor_x;
    const float dz = pt.z - sensor_z;
    if (dx * dx + dz * dz > max_range2) continue;  // fuera de rango

    // Celda del obstáculo.
    int obs_ci, obs_cj;
    if (!worldToCell(pt.x, pt.z, obs_ci, obs_cj)) continue;

    // ── Observación ocupada ──────────────────────────────────────────────────
    updateCell(obs_ci, obs_cj, cfg_.l_occ);

    // ── Ray casting: observación libre desde el sensor hasta el obstáculo ───
    if (cfg_.enable_raycasting) {
      castRay(sensor_ci, sensor_cj, obs_ci, obs_cj,
              [this, &free_updated](int ci, int cj) {
                const auto idx = static_cast<std::size_t>(ci * cfg_.grid_size + cj);
                if (!free_updated[idx]) {
                  updateCell(ci, cj, cfg_.l_free);
                  free_updated[idx] = true;
                }
              });
    }
  }

  // ── Ray casting para rayos libres (sin obstáculo al final) ─────────────────
  // Cubre dos casos que el bucle anterior no atiende:
  //   1. Puntos más allá del rango máximo: el sensor midió z >= max_range_m;
  //      no hay obstáculo en ese rayo dentro del rango de interés.
  //   2. Puntos de suelo o techo: válidos en [min,max]_range pero no son
  //      obstáculos para navegación; las celdas hasta ellos deben ser libres.
  // Para cada endpoint se castea un rayo libre hasta la celda final inclusive
  // (a diferencia del caso de obstáculo, donde la celda final es ocupada).
  for (const auto& pt : free_ray_endpoints) {
    const float dx = pt.x - sensor_x;
    const float dz = pt.z - sensor_z;
    if (dx * dx + dz * dz > max_range2) continue;  // demasiado lejos: ignorar

    int end_ci, end_cj;
    if (!worldToCell(pt.x, pt.z, end_ci, end_cj)) continue;  // fuera del grid

    if (cfg_.enable_raycasting) {
      // Celdas intermedias (excluye endpoint).
      castRay(sensor_ci, sensor_cj, end_ci, end_cj,
              [this, &free_updated](int ci, int cj) {
                const auto idx =
                    static_cast<std::size_t>(ci * cfg_.grid_size + cj);
                if (!free_updated[idx]) {
                  updateCell(ci, cj, cfg_.l_free);
                  free_updated[idx] = true;
                }
              });
      // Celda del endpoint: también libre (no hay obstáculo aquí).
      const auto end_idx =
          static_cast<std::size_t>(end_ci * cfg_.grid_size + end_cj);
      if (!free_updated[end_idx]) {
        updateCell(end_ci, end_cj, cfg_.l_free);
        free_updated[end_idx] = true;
      }
    }
  }
}

}  // namespace local_mapper
