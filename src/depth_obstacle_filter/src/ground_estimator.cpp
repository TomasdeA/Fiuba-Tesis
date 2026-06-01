#include "depth_obstacle_filter/ground_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <future>
#include <numeric>
#include <random>
#include <stdexcept>
#include <unordered_map>
#include <time.h>

namespace depth_obstacle_filter {

// ═════════════════════════════════════════════════════════════════════════════
// Utilidades internas
// ═════════════════════════════════════════════════════════════════════════════

float GroundEstimator::elapsedMs(struct timespec start) {
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  return static_cast<float>(
      (now.tv_sec - start.tv_sec) * 1000.0 +
      (now.tv_nsec - start.tv_nsec) / 1e6);
}

float GroundEstimator::pointPlaneDist(const Point3D& p, const Plane& pl) {
  return pl.nx * p.x + pl.ny * p.y + pl.nz * p.z + pl.d;
}

// Ajuste de plano por mínimos cuadrados (SVD implícito via método de covarianza).
// Dado N puntos, calcula centroide, forma la matriz de covarianza 3×3 y
// encuentra el eigenvector al eigenvalor mínimo (normal del plano).
// Implementación sin dependencias externas usando el método de Jacobi 3×3.
GroundEstimator::Plane GroundEstimator::fitPlaneLS(
    const std::vector<const Point3D*>& pts)
{
  Plane result;
  if (pts.size() < 3) { return result; }

  // Centroide
  double cx = 0, cy = 0, cz = 0;
  for (const auto* p : pts) { cx += p->x; cy += p->y; cz += p->z; }
  const double n = static_cast<double>(pts.size());
  cx /= n; cy /= n; cz /= n;

  // Matriz de covarianza (simétrica 3×3)
  double cov[3][3] = {};
  for (const auto* p : pts) {
    const double dx = p->x - cx, dy = p->y - cy, dz = p->z - cz;
    cov[0][0] += dx*dx; cov[0][1] += dx*dy; cov[0][2] += dx*dz;
    cov[1][1] += dy*dy; cov[1][2] += dy*dz;
    cov[2][2] += dz*dz;
  }
  cov[1][0] = cov[0][1]; cov[2][0] = cov[0][2]; cov[2][1] = cov[1][2];

  // Algoritmo iterativo de Jacobi para hallar eigenvectores de matriz 3×3
  // (suficiente para 3 iteraciones, converge rápido en casos bien condicionados)
  double V[3][3] = {{1,0,0},{0,1,0},{0,0,1}};  // eigenvectores (columnas)
  double A[3][3];
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      A[i][j] = cov[i][j];

  for (int sweep = 0; sweep < 12; sweep++) {
    for (int p = 0; p < 2; p++) {
      for (int q = p+1; q < 3; q++) {
        if (std::abs(A[p][q]) < 1e-12) continue;
        double theta = 0.5 * std::atan2(2.0*A[p][q], A[p][p]-A[q][q]);
        double c = std::cos(theta), s = std::sin(theta);
        // Aplicar rotación de Jacobi
        double Ap = c*c*A[p][p] + 2*s*c*A[p][q] + s*s*A[q][q];
        double Aq = s*s*A[p][p] - 2*s*c*A[p][q] + c*c*A[q][q];
        double Apq = 0.0;
        A[p][p] = Ap; A[q][q] = Aq; A[p][q] = A[q][p] = Apq;
        // Actualización del resto de la matriz
        for (int r = 0; r < 3; r++) {
          if (r == p || r == q) continue;
          double Arp = c*A[r][p] + s*A[r][q];
          double Arq = -s*A[r][p] + c*A[r][q];
          A[r][p] = A[p][r] = Arp;
          A[r][q] = A[q][r] = Arq;
        }
        // Acumulación de autovectores
        for (int r = 0; r < 3; r++) {
          double Vrp = c*V[r][p] + s*V[r][q];
          double Vrq = -s*V[r][p] + c*V[r][q];
          V[r][p] = Vrp; V[r][q] = Vrq;
        }
      }
    }
  }

  // Eigenvector al eigenvalor mínimo = normal del plano
  int min_idx = 0;
  for (int i = 1; i < 3; i++)
    if (A[i][i] < A[min_idx][min_idx]) min_idx = i;

  double nx = V[0][min_idx], ny = V[1][min_idx], nz = V[2][min_idx];
  double norm = std::sqrt(nx*nx + ny*ny + nz*nz);
  if (norm < 1e-9) { return result; }
  nx /= norm; ny /= norm; nz /= norm;

  // Convención: normal apunta hacia Y negativo (hacia arriba en frame alineado)
  if (ny > 0) { nx = -nx; ny = -ny; nz = -nz; }

  result.nx = static_cast<float>(nx);
  result.ny = static_cast<float>(ny);
  result.nz = static_cast<float>(nz);
  result.d  = -static_cast<float>(nx*cx + ny*cy + nz*cz);
  result.valid = true;
  return result;
}

// ═════════════════════════════════════════════════════════════════════════════
// Constructor
// ═════════════════════════════════════════════════════════════════════════════

GroundEstimator::GroundEstimator(const Config& cfg) : cfg_(cfg) {}

// ═════════════════════════════════════════════════════════════════════════════
// Pipeline principal
// ═════════════════════════════════════════════════════════════════════════════

bool GroundEstimator::estimate(const std::vector<Point3D>& cloud) {
  const bool collect_perf = cfg_.collect_perf_stats;
  ground_plane_ = Plane{};
  labels_.assign(cloud.size(), Label::UNKNOWN);
  ground_idx_.clear();
  obstacle_idx_.clear();
  ceiling_idx_.clear();
  perf_ = PerfStats{};
  perf_.n_input = static_cast<int>(cloud.size());

  if (cloud.size() < 10) return false;

  struct timespec t_total;
  if (collect_perf) clock_gettime(CLOCK_MONOTONIC, &t_total);

  // ── Etapas 1 y 2 en paralelo ─────────────────────────────────────────────
  // stage1_diagnostics y stage2_voxel_sample son const y solo leen 'cloud',
  // sin estado mutable compartido — se lanzan en hilos independientes.
  // Cada lambda mide su propio tiempo para no mezclar las métricas.
  auto fut_diag = std::async(std::launch::async, [this, &cloud, collect_perf] {
    struct timespec t;
    if (collect_perf) clock_gettime(CLOCK_MONOTONIC, &t);
    auto s = stage1_diagnostics(cloud);
    return std::make_pair(s, collect_perf ? elapsedMs(t) : 0.0f);
  });
  auto fut_vox = std::async(std::launch::async, [this, &cloud, collect_perf] {
    struct timespec t;
    if (collect_perf) clock_gettime(CLOCK_MONOTONIC, &t);
    auto v = stage2_voxel_sample(cloud);
    return std::make_pair(std::move(v), collect_perf ? elapsedMs(t) : 0.0f);
  });

  auto [stats, t_diag] = fut_diag.get();
  stats_ = stats;
  perf_.time_diag_ms = t_diag;

  auto [voxel_cloud, t_vox] = fut_vox.get();
  perf_.time_voxel_ms = t_vox;
  perf_.n_voxel = static_cast<int>(voxel_cloud.size());

  // Tolerancia: se calcula antes de RANSAC para poder usarla en stage7
  // aunque el pipeline falle en etapas posteriores.
  const float tol = (cfg_.ransac_inlier_tol > 0.0f)
      ? cfg_.ransac_inlier_tol
      : stage3_adaptive_tolerance(stats_);

  // stage7 SIEMPRE se ejecuta al finalizar, incluso si no se detectó un plano.
  // Si ground_plane_.valid == false, no se asigna GROUND pero sí CEILING y
  // OBSTACLE, de modo que obstacle_idx_ está siempre poblado para el downstream.
  auto finalize = [&](bool ok) -> bool {
    stage7_classify(cloud, ground_plane_, tol);
    perf_.n_inliers = ground_plane_.n_inliers;
    if (collect_perf) perf_.time_total_ms = elapsedMs(t_total);
    return ok;
  };

  if (voxel_cloud.size() < 10) return finalize(false);

  // Fast path: si el plano del frame anterior todavía explica bien la nube,
  // no se vuelve a buscar un plano aleatorio. En una cámara montada en la
  // cabeza, la altura al piso cambia lento; esto evita jitter y CPU innecesaria.
  if (cfg_.enable_plane_cache && last_good_plane_.valid) {
    ground_plane_ = scoreCachedPlane(cloud, tol);
    if (ground_plane_.valid) {
      if (collect_perf) perf_.time_total_ms = elapsedMs(t_total);
      perf_.n_inliers = ground_plane_.n_inliers;
      perf_.used_cached_plane = true;
      stage7_classify(cloud, ground_plane_, tol);
      return true;
    }
    ground_plane_ = Plane{};
  }

  // ── Etapa 4: RANSAC jerárquico ────────────────────────────────────────────
  struct timespec t_ransac;
  if (collect_perf) clock_gettime(CLOCK_MONOTONIC, &t_ransac);
  Plane candidate = stage4_ransac(voxel_cloud, tol, cfg_.ransac_max_iter);
  if (collect_perf) perf_.time_ransac_ms = elapsedMs(t_ransac);
  perf_.ransac_iterations = cfg_.ransac_max_iter;

  if (!candidate.valid)            return finalize(false);

  // ── Etapa 5: Validación geométrica ────────────────────────────────────────
  if (!stage5_validate(candidate)) return finalize(false);

  // ── Etapa 6: Refinamiento por minimos cuadrados (LS) sobre inliers de la ──
  // ─────────── nube completa ────────────────────────────────────────────────
  struct timespec t_ref;
  if (collect_perf) clock_gettime(CLOCK_MONOTONIC, &t_ref);
  ground_plane_ = stage6_refine(cloud, candidate, tol);
  if (collect_perf) perf_.time_refine_ms = elapsedMs(t_ref);

  if (!ground_plane_.valid)        return finalize(false);

  // ── Etapas 7+8: Clasificación y performance ───────────────────────────────
  last_good_plane_ = ground_plane_;
  return finalize(true);
}

// ═════════════════════════════════════════════════════════════════════════════
// Etapa 1: Diagnóstico
// ═════════════════════════════════════════════════════════════════════════════

GroundEstimator::CloudStats GroundEstimator::stage1_diagnostics(
    const std::vector<Point3D>& cloud) const
{
  CloudStats s;
  s.n_total = static_cast<int>(cloud.size());
  if (cloud.empty()) return s;

  // Muestrear un subconjunto para estimar distancia al vecino más cercano.
  // Búsqueda naïve O(n²) sobre la muestra; suficiente para n_sample ≤ 300.
  const int n = static_cast<int>(cloud.size());
  const int n_sample = std::min(cfg_.nn_sample_size, n);

  // Índices muestreados uniformemente
  std::vector<int> idx(n);
  std::iota(idx.begin(), idx.end(), 0);
  // Usar semilla fija para reproducibilidad entre frames
  std::mt19937 rng(42);
  std::shuffle(idx.begin(), idx.end(), rng);
  idx.resize(n_sample);

  std::vector<float> nn_dists;
  nn_dists.reserve(n_sample);

  // Búsqueda exhaustiva del vecino más cercano: O(n_sample²).
  // Para cada punto pi de la muestra, recorre todos los demás pj y guarda
  // la distancia al cuadrado mínima (evita la raíz cuadrada en el inner loop).
  // Al salir del inner loop se aplica sqrt una sola vez.
  for (int i = 0; i < n_sample; i++) {
    const auto& pi = cloud[idx[i]];
    float min_d2 = std::numeric_limits<float>::max();
    for (int j = 0; j < n_sample; j++) {
      if (i == j) continue;
      const auto& pj = cloud[idx[j]];
      const float dx = pi.x-pj.x, dy = pi.y-pj.y, dz = pi.z-pj.z;
      min_d2 = std::min(min_d2, dx*dx + dy*dy + dz*dz);
    }
    // Solo registrar si se encontró al menos un vecino (guarda consistencia
    // si n_sample == 1, aunque ese caso se descarta antes por cloud.size() < 10)
    if (min_d2 < std::numeric_limits<float>::max())
      nn_dists.push_back(std::sqrt(min_d2));
  }

  // Si por alguna razón la muestra quedó vacía, devolver estadísticas nulas
  if (nn_dists.empty()) return s;

  // Media: distancia promedio al vecino más cercano (proxy de densidad local).
  // Varianza: dispersión de esas distancias (proxy de ruido del sensor).
  const float mean = std::accumulate(nn_dists.begin(), nn_dists.end(), 0.0f)
                     / static_cast<float>(nn_dists.size());
  float var = 0;
  for (float d : nn_dists) var += (d-mean)*(d-mean);
  var /= static_cast<float>(nn_dists.size());
  const float std_dev = std::sqrt(var);

  s.mean_nn_dist   = mean;
  s.std_nn_dist    = std_dev;
  // noise_estimate ≈ σ_NN: se usa en stage3 para calcular la tolerancia
  // adaptativa ε = 3·σ_NN (intervalo de confianza del 99.7% gaussiano)
  s.noise_estimate = std_dev;

  // ── Densidad volumétrica ─────────────────────────────────────────────────
  // Estimación gruesa: n_total / volumen de la bounding box de la nube completa.
  float xmin = cloud[0].x, xmax = cloud[0].x;
  float ymin = cloud[0].y, ymax = cloud[0].y;
  float zmin = cloud[0].z, zmax = cloud[0].z;
  for (const auto& p : cloud) {
    xmin = std::min(xmin, p.x); xmax = std::max(xmax, p.x);
    ymin = std::min(ymin, p.y); ymax = std::max(ymax, p.y);
    zmin = std::min(zmin, p.z); zmax = std::max(zmax, p.z);
  }
  // Clamp a 0e-6 evita división por cero en nubes degeneradas (plano exacto).
  const float vol = std::max(1e-6f,
      (xmax-xmin) * (ymax-ymin) * (zmax-zmin));
  s.density = static_cast<float>(n) / vol;

  // ── Detección de outliers ────────────────────────────────────────────────
  // Un punto se considera outlier si su distancia NN supera mean + 3·σ
  // Se cuenta sobre la muestra (no sobre la nube completa) por eficiencia.
  const float outlier_thresh = mean + 3.0f * std_dev;
  s.n_outliers = 0;
  for (float d : nn_dists)
    if (d > outlier_thresh) s.n_outliers++;

  s.valid = true;
  return s;
}

// ═════════════════════════════════════════════════════════════════════════════
// Etapa 2: Voxel sampling
// ═════════════════════════════════════════════════════════════════════════════

std::vector<GroundEstimator::Point3D> GroundEstimator::stage2_voxel_sample(
    const std::vector<Point3D>& cloud) const
{
  if (!cfg_.enable_voxel_filter) return cloud;
  if (cloud.empty()) return {};
  const float inv_vs = 1.0f / cfg_.voxel_size_m;

  // Acumular puntos por celda voxel y promediar
  using Key = std::array<int32_t, 3>;
  struct Hash {
    size_t operator()(const Key& k) const {
      size_t h = 0;
      for (auto v : k) h ^= std::hash<int32_t>{}(v) + 0x9e3779b9 + (h<<6) + (h>>2);
      return h;
    }
  };
  std::unordered_map<Key, std::array<float, 4>, Hash> cells;

  for (const auto& p : cloud) {
    // Puntos de techo (Y < -ceiling_delta_m) se descartarán en Stage 7;
    // excluirlos aquí evita computar floor+hash para puntos que no aportan
    // al plano del suelo ni a los obstáculos.
    if (p.y < -cfg_.ceiling_delta_m) continue;
    Key k = {
      static_cast<int32_t>(std::floor(p.x * inv_vs)),
      static_cast<int32_t>(std::floor(p.y * inv_vs)),
      static_cast<int32_t>(std::floor(p.z * inv_vs))
    };
    auto& acc = cells[k];
    acc[0] += p.x; acc[1] += p.y; acc[2] += p.z; acc[3] += 1.0f;
  }

  std::vector<Point3D> result;
  result.reserve(cells.size());
  for (const auto& [k, acc] : cells) {
    (void)k;
    const float inv_n = 1.0f / acc[3];
    result.push_back({acc[0]*inv_n, acc[1]*inv_n, acc[2]*inv_n});
  }
  return result;
}

// ═════════════════════════════════════════════════════════════════════════════
// Etapa 3: Tolerancia adaptativa
// ═════════════════════════════════════════════════════════════════════════════

float GroundEstimator::stage3_adaptive_tolerance(const CloudStats& stats) const {
  if (!stats.valid || stats.noise_estimate < 1e-6f) {
    // Fallback conservador: 3 cm
    return 0.03f;
  }
  // 3 × ruido estimado, acotado entre 1 cm y 10 cm para estabilidad
  return std::clamp(3.0f * stats.noise_estimate, 0.01f, 0.10f);
}

GroundEstimator::Plane GroundEstimator::scoreCachedPlane(
    const std::vector<Point3D>& cloud, float tol) const
{
  if (!last_good_plane_.valid || cloud.empty()) return Plane{};

  Plane scored = last_good_plane_;
  int floor_candidates = 0;
  int inliers = 0;
  for (const auto& p : cloud) {
    if (p.y < cfg_.min_person_height_m) continue;
    ++floor_candidates;
    const float r = std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
    const float eps = std::max(tol, cfg_.floor_noise_k * r);
    if (std::abs(pointPlaneDist(p, last_good_plane_)) < eps) {
      ++inliers;
    }
  }

  if (floor_candidates < cfg_.cached_plane_min_inliers) return Plane{};

  scored.n_inliers = inliers;
  scored.quality = static_cast<float>(inliers) /
                   static_cast<float>(floor_candidates);
  scored.valid =
      scored.quality >= cfg_.cached_plane_min_quality &&
      inliers >= cfg_.cached_plane_min_inliers &&
      stage5_validate(scored);
  return scored;
}

// ═════════════════════════════════════════════════════════════════════════════
// Etapa 4: RANSAC jerárquico
// ═════════════════════════════════════════════════════════════════════════════
//
// Filtro de candidatos por histograma de Y:
//   Primero se seleccionan puntos con Y >= min_person_height_m (cota inferior
//   del suelo en el frame alineado). Sobre esa franja se construye un
//   histograma de Y con bins de hist_bin_m. El bin más poblado localiza el
//   nivel dominante de la escena; la banda
//   [y_pico - hist_band_low_m, y_pico + hist_band_high_m] es el pool de
//   muestreo. Usar solo esa banda evita que paredes y obstáculos compitan
//   con el suelo durante el muestreo.
//
// Métrica de calidad relativa a la banda:
//   quality = inliers_banda / n_banda. Con el denominador n_total la calidad
//   quedaría diluida cuando el suelo ocupa una fracción pequeña de la nube
//   (cámara apuntando al frente). El umbral ransac_min_inliers se interpreta
//   sobre la banda, no sobre la nube completa.
//
// Tolerancia variable ε_i = max(ε, k·‖p_i‖):
//   El ruido axial del D435 crece con la distancia; la componente
//   perpendicular al plano del suelo escala aproximadamente como k·‖p‖
//   (k ≈ 0,015). La tolerancia global ε, calculada en Stage 3 a partir de
//   la distribución NN de la nube, representa el nivel de ruido cercano;
//   para puntos lejanos se usa k·‖p‖ cuando éste es mayor que ε.

GroundEstimator::Plane GroundEstimator::stage4_ransac(
    const std::vector<Point3D>& cloud, float tol, int max_iter) const
{
  const int n = static_cast<int>(cloud.size());
  if (n < 3) return Plane{};

  // Primer filtro: puntos con Y >= min_person_height_m son candidatos a suelo.
  // La cámara se monta en la cabeza; en el frame alineado Y=0 es la cámara
  // e Y crece hacia abajo. El suelo siempre está a Y >= min_person_height_m.
  std::vector<int> floor_idx;
  floor_idx.reserve(n);
  for (int i = 0; i < n; ++i) {
    if (cloud[i].y >= cfg_.min_person_height_m)
      floor_idx.push_back(i);
  }

  // Histograma de Y sobre los candidatos: localizar el nivel del suelo.
  // El pico es la superficie horizontal más densa; la banda
  // [y_pico - hist_band_low_m, y_pico + hist_band_high_m] concentra los
  // puntos del suelo y excluye obstáculos y paredes del pool de RANSAC.
  std::vector<int> band_idx;
  if (static_cast<int>(floor_idx.size()) >= 3) {
    float y_lo = std::numeric_limits<float>::max();
    float y_hi = std::numeric_limits<float>::lowest();
    for (int i : floor_idx) {
      y_lo = std::min(y_lo, cloud[i].y);
      y_hi = std::max(y_hi, cloud[i].y);
    }
    const float bin  = cfg_.hist_bin_m;
    const int n_bins = std::max(1, static_cast<int>((y_hi - y_lo) / bin) + 1);
    std::vector<int> hist(n_bins, 0);
    for (int i : floor_idx) {
      int b = static_cast<int>((cloud[i].y - y_lo) / bin);
      b = std::clamp(b, 0, n_bins - 1);
      hist[b]++;
    }
    const int   peak_bin = static_cast<int>(
        std::max_element(hist.begin(), hist.end()) - hist.begin());
    const float y_peak   = y_lo + (peak_bin + 0.5f) * bin;

    const float y_band_lo = y_peak - cfg_.hist_band_low_m;
    const float y_band_hi = y_peak + cfg_.hist_band_high_m;
    band_idx.reserve(floor_idx.size());
    for (int i : floor_idx) {
      if (cloud[i].y >= y_band_lo && cloud[i].y <= y_band_hi)
        band_idx.push_back(i);
    }
  }

  // Fallback: si la banda queda vacía (suelo no visible o nube degenerada),
  // usar floor_idx para no rechazar la búsqueda prematuramente.
  if (static_cast<int>(band_idx.size()) < 3)
    band_idx = floor_idx;

  // El costo dominante de RANSAC es contar inliers para cada candidato. Cuando
  // no hay voxel, la banda puede contener decenas de miles de puntos; limitarla
  // mantiene latencia acotada sin cambiar el pool geométrico de candidatos.
  const int max_band_points = cfg_.ransac_max_band_points;
  if (max_band_points > 0 &&
      static_cast<int>(band_idx.size()) > max_band_points) {
    std::vector<int> limited;
    limited.reserve(max_band_points);
    const double step = static_cast<double>(band_idx.size()) /
                        static_cast<double>(max_band_points);
    for (int i = 0; i < max_band_points; ++i) {
      const int src = std::min(
          static_cast<int>(i * step),
          static_cast<int>(band_idx.size()) - 1);
      limited.push_back(band_idx[src]);
    }
    band_idx = std::move(limited);
  }

  const int n_band = static_cast<int>(band_idx.size());
  if (n_band < 3) return Plane{};  // suelo no visible en la escena

  Plane best;
  best.quality = -1.0f;
  std::mt19937 rng(12345);

  // Cuenta inliers sobre band_idx con tolerancia variable ε_i = max(ε, k·‖p_i‖).
  // quality = cnt / n_band (denominador relativo a la banda, no a la nube total).
  auto count_band_inliers = [&](const Plane& cand) -> int {
    int cnt = 0;
    for (int bi : band_idx) {
      const auto& p   = cloud[bi];
      const float r   = std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
      const float eps = std::max(tol, cfg_.floor_noise_k * r);
      if (std::abs(pointPlaneDist(p, cand)) < eps) cnt++;
    }
    return cnt;
  };

  // Fase 1: muestrea desde band_idx (70 % de las iteraciones).
  // Fase 2: si la calidad no alcanza el umbral, amplía a floor_idx completo.
  const int iter_phase1 = static_cast<int>(max_iter * 0.7f);
  const int iter_phase2 = max_iter - iter_phase1;

  auto do_ransac = [&](const std::vector<int>& pool, int iters) {
    if (static_cast<int>(pool.size()) < 3) return;
    std::uniform_int_distribution<int> dist(0, static_cast<int>(pool.size())-1);
    for (int it = 0; it < iters; it++) {
      int i0 = pool[dist(rng)];
      int i1 = pool[dist(rng)];
      int i2 = pool[dist(rng)];
      if (i0 == i1 || i0 == i2 || i1 == i2) continue;

      const auto& p0 = cloud[i0];
      const auto& p1 = cloud[i1];
      const auto& p2 = cloud[i2];
      float ax = p1.x-p0.x, ay = p1.y-p0.y, az = p1.z-p0.z;
      float bx = p2.x-p0.x, by = p2.y-p0.y, bz = p2.z-p0.z;
      float nx = ay*bz - az*by;
      float ny = az*bx - ax*bz;
      float nz = ax*by - ay*bx;
      float nm = std::sqrt(nx*nx + ny*ny + nz*nz);
      if (nm < 1e-8f) continue;
      nx /= nm; ny /= nm; nz /= nm;
      float d = -(nx*p0.x + ny*p0.y + nz*p0.z);
      if (ny > 0) { nx=-nx; ny=-ny; nz=-nz; d=-d; }

      Plane cand; cand.nx=nx; cand.ny=ny; cand.nz=nz; cand.d=d; cand.valid=true;

      const int   cnt = count_band_inliers(cand);
      const float q   = static_cast<float>(cnt) / static_cast<float>(n_band);
      if (q > best.quality) {
        best           = cand;
        best.quality   = q;
        best.n_inliers = cnt;
      }
    }
  };

  do_ransac(band_idx, iter_phase1);

  if (best.quality < cfg_.ransac_min_inliers)
    do_ransac(floor_idx, iter_phase2);

  if (best.quality < cfg_.ransac_min_inliers) best.valid = false;
  return best;
}

// ═════════════════════════════════════════════════════════════════════════════
// Etapa 5: Validación geométrica
// ═════════════════════════════════════════════════════════════════════════════

bool GroundEstimator::stage5_validate(const Plane& plane) const {
  if (!plane.valid) return false;
  if (plane.quality < cfg_.min_quality) return false;

  // El plano del suelo debe ser mayormente horizontal:
  // su normal debe estar cerca del eje Y (|ny| grande)
  // Ángulo entre la normal y el eje Y: cos(α) = |ny|
  const float cos_tilt = std::abs(plane.ny);
  const float tilt_deg = std::acos(std::clamp(cos_tilt, 0.0f, 1.0f))
                         * 180.0f / static_cast<float>(M_PI);
  return tilt_deg < cfg_.max_tilt_deg;
}

// ═════════════════════════════════════════════════════════════════════════════
// Etapa 6: Refinamiento por mínimos cuadrados
// ═════════════════════════════════════════════════════════════════════════════

GroundEstimator::Plane GroundEstimator::stage6_refine(
    const std::vector<Point3D>& cloud,
    const Plane& initial, float tol) const
{
  // Colectar inliers de la nube completa con la tolerancia extendida (1.5×)
  const float tol_refine = tol * 1.5f;
  std::vector<const Point3D*> inliers;
  inliers.reserve(cloud.size() / 4);
  for (const auto& p : cloud)
    if (std::abs(pointPlaneDist(p, initial)) < tol_refine)
      inliers.push_back(&p);

  if (inliers.size() < 3) return initial;

  Plane refined = fitPlaneLS(inliers);
  if (!refined.valid) return initial;

  // n_inliers: conteo real sobre la nube completa con el plano refinado.
  // quality: se conserva el valor calculado en Stage 4, que usa n_banda como
  // denominador. Recalcularlo con n_total aquí produciría un denominador
  // inconsistente con el umbral min_quality de Stage 5.
  int cnt = 0;
  for (const auto& p : cloud)
    if (std::abs(pointPlaneDist(p, refined)) < tol) cnt++;
  refined.n_inliers = cnt;
  refined.quality   = initial.quality;
  return refined;
}

// ═════════════════════════════════════════════════════════════════════════════
// Etapa 7: Clasificación
// ═════════════════════════════════════════════════════════════════════════════
//
//  Cada punto recibe una de tres etiquetas:
//
//  GROUND   – dentro de la tolerancia del plano del suelo detectado, o en el
//             semiespacio por debajo de ese plano (distancia firmada negativa).
//  CEILING  – Y < -ceiling_delta_m: está por encima de la cámara más de
//             ceiling_delta_m. Como la cámara se monta en la cabeza,
//             estos puntos están sobre el techo o por encima del usuario
//             y no aportan información relevante al mapa; se descartan
//             (no se añaden a obstacle_idx_).
//  OBSTACLE – todo lo demás (paredes, objetos, cuerpo del usuario, etc.).

void GroundEstimator::stage7_classify(
    const std::vector<Point3D>& cloud,
    const Plane& plane, float tol)
{
  labels_.resize(cloud.size(), Label::UNKNOWN);
  ground_idx_.clear();
  obstacle_idx_.clear();
  ceiling_idx_.clear();

  for (int i = 0; i < static_cast<int>(cloud.size()); i++) {
    // GROUND: dentro de tolerancia o por debajo del plano de piso.
    // Con la convención de normal (ny < 0), distancias firmadas negativas
    // corresponden a puntos por debajo del plano (hacia +Y).
    const float signed_dist = pointPlaneDist(cloud[i], plane);
    if (plane.valid && (signed_dist < 0.0f || std::abs(signed_dist) < tol)) {
      labels_[i] = Label::GROUND;
      ground_idx_.push_back(i);
      continue;
    }

    // CEILING: más de ceiling_delta_m por encima de la cámara (Y negativo)
    if (cloud[i].y < -cfg_.ceiling_delta_m) {
      labels_[i] = Label::CEILING;
      ceiling_idx_.push_back(i);
      continue;  // no se añade al mapa de obstáculos
    }

    // OBSTACLE: todo lo demás (obstáculos chocables, paredes, etc.)
    labels_[i] = Label::OBSTACLE;
    obstacle_idx_.push_back(i);
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// Getter: altura de la cámara
// ═════════════════════════════════════════════════════════════════════════════

float GroundEstimator::cameraHeightM() const {
  if (!ground_plane_.valid) return 0.0f;
  // El origen de la cámara (0,0,0) está a distancia |d| del plano
  // Signo: d > 0 → origen por encima del plano (cámara sobre el suelo)
  return ground_plane_.d;
}

}  // namespace depth_obstacle_filter
