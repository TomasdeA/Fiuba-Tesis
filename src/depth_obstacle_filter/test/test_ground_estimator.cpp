#include <gtest/gtest.h>
#include <cmath>
#include <vector>

#include "depth_obstacle_filter/ground_estimator.hpp"

using GE = depth_obstacle_filter::GroundEstimator;
using P3 = GE::Point3D;

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

/// Genera una nube de puntos distribuidos aleatoriamente sobre un plano
/// y = y_val (frame alineado a gravedad: Y apunta hacia abajo).
/// Se añade ruido gaussiano de amplitud noise_m en Y.
static std::vector<P3> makeFlatCloud(int n, float y_val, float noise_m = 0.005f) {
  std::vector<P3> cloud;
  cloud.reserve(n + 50);

  // Semilla fija para reproducibilidad
  uint32_t seed = 0x1234ABCD;
  auto rng = [&]() -> float {
    seed ^= seed << 13; seed ^= seed >> 17; seed ^= seed << 5;
    return static_cast<float>(seed & 0xFFFF) / 65535.0f;  // [0, 1]
  };

  for (int i = 0; i < n; i++) {
    const float x = (rng() - 0.5f) * 4.0f;
    const float z = rng() * 3.0f + 0.5f;
    const float y = y_val + (rng() - 0.5f) * 2.0f * noise_m;
    cloud.push_back({x, y, z});
  }

  // Algunos puntos "obstáculo" por encima del plano (Y < y_val)
  for (int i = 0; i < 50; i++) {
    cloud.push_back({(rng()-0.5f)*2.0f, y_val - 0.5f - rng()*0.5f, rng()*2.0f + 0.5f});
  }
  return cloud;
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

TEST(GroundEstimator, DetectaPlanoHorizontal) {
  // Suelo plano en y = +1.5 m (cámara a ~1.5 m sobre el suelo en frame alineado).
  GE estimator;
  const auto cloud = makeFlatCloud(800, 1.5f, 0.003f);
  ASSERT_TRUE(estimator.estimate(cloud));
  EXPECT_TRUE(estimator.isValid());

  const auto& plane = estimator.groundPlane();
  // La normal debe ser casi paralela al eje Y: |ny| cercano a 1
  EXPECT_GT(std::abs(plane.ny), 0.95f)
      << "Normal Y: " << plane.ny;
  // Calidad mínima esperada
  EXPECT_GT(plane.quality, 0.5f)
      << "Quality: " << plane.quality;
  // Hay inliers clasificados como GROUND
  EXPECT_GT(static_cast<int>(estimator.groundIndices().size()), 100);
}

TEST(GroundEstimator, RechazaPlanoMuyInclinado) {
  // Plano vertical: normal ≈ (1,0,0) → debe fallar validación Stage 5
  GE::Config cfg;
  cfg.max_tilt_deg = 15.0f;
  GE estimator(cfg);

  std::vector<P3> cloud;
  // Plano vertical x = 1.5, Y muy disperso
  uint32_t seed = 42;
  auto rng = [&]() -> float {
    seed ^= seed << 13; seed ^= seed >> 17; seed ^= seed << 5;
    return static_cast<float>(seed & 0xFFFF) / 65535.0f;
  };
  for (int i = 0; i < 500; i++) {
    cloud.push_back({1.5f + (rng()-0.5f)*0.01f,
                     (rng()-0.5f)*3.0f,
                     rng()*2.0f + 0.5f});
  }
  // No debería encontrar un suelo plano
  EXPECT_FALSE(estimator.estimate(cloud));
}

TEST(GroundEstimator, VoxelSamplingReduceNubeDensa) {
  // Nube muy densa: el voxel sampling debe reducirla considerablemente
  GE::Config cfg;
  cfg.voxel_size_m = 0.1f;
  GE estimator(cfg);

  // Generar nube uniforme muy densa
  std::vector<P3> cloud;
  cloud.reserve(10000);
  for (int xi = 0; xi < 100; xi++) {
    for (int zi = 0; zi < 100; zi++) {
      cloud.push_back({
        static_cast<float>(xi) * 0.02f - 1.0f,
        0.8f,
        static_cast<float>(zi) * 0.02f + 0.3f
      });
    }
  }

  estimator.estimate(cloud);
  const int n_voxel = estimator.perfStats().n_voxel;
  const int n_input = estimator.perfStats().n_input;
  // El downsampling debe reducir al menos 4× para voxel 0.1 en rejilla 0.02
  EXPECT_LT(n_voxel, n_input / 4)
      << "n_voxel=" << n_voxel << " n_input=" << n_input;
}

TEST(GroundEstimator, NubeVaciaNoFalla) {
  GE estimator;
  std::vector<P3> empty;
  EXPECT_NO_THROW(estimator.estimate(empty));
  EXPECT_FALSE(estimator.isValid());
}

TEST(GroundEstimator, NubePequenaNoFalla) {
  GE estimator;
  std::vector<P3> tiny = {{0,0,1},{0.1f,0,1.1f},{0.2f,0,0.9f}};
  EXPECT_NO_THROW(estimator.estimate(tiny));
  EXPECT_FALSE(estimator.isValid());
}

TEST(GroundEstimator, DiagnosticoCalculaEstadisticas) {
  GE estimator;
  const auto cloud = makeFlatCloud(400, 1.5f, 0.005f);
  estimator.estimate(cloud);

  const auto& stats = estimator.cloudStats();
  EXPECT_TRUE(stats.valid);
  EXPECT_GT(stats.mean_nn_dist, 0.0f);
  EXPECT_GE(stats.n_total, 400);  // Al menos los puntos del suelo
}

TEST(GroundEstimator, ClasificacionSueloYObstaculo) {
  GE estimator;
  const auto cloud = makeFlatCloud(800, 1.5f, 0.003f);
  const bool ok = estimator.estimate(cloud);
  if (!ok) GTEST_SKIP() << "Estimación no exitosa en esta configuración";

  // Todos los índices dentro de rango
  for (int idx : estimator.groundIndices()) {
    EXPECT_GE(idx, 0);
    EXPECT_LT(idx, static_cast<int>(cloud.size()));
  }
  for (int idx : estimator.obstacleIndices()) {
    EXPECT_GE(idx, 0);
    EXPECT_LT(idx, static_cast<int>(cloud.size()));
  }

  // Los 50 puntos obstáculo deberían estar clasificados como obstáculo
  EXPECT_GT(static_cast<int>(estimator.obstacleIndices().size()), 20);
}

TEST(GroundEstimator, PerformanceDentroDePresupuesto) {
  // El pipeline completo debe correr en < 100 ms para nube de 5000 puntos
  GE estimator;
  const auto cloud = makeFlatCloud(5000, 1.5f, 0.005f);
  estimator.estimate(cloud);

  const float t = estimator.perfStats().time_total_ms;
  EXPECT_LT(t, 100.0f)
      << "Pipeline tardó " << t << " ms para 5000 puntos";
}

TEST(GroundEstimator, AlturaDeCamera) {
  // El origen de la cámara está en (0,0,0). El plano del suelo está en y = 1.5.
  // La normal apunta hacia Y negativo (arriba), por lo que d ≈ +1.5.
  GE estimator;
  const auto cloud = makeFlatCloud(800, 1.5f, 0.003f);
  const bool ok = estimator.estimate(cloud);
  if (!ok) GTEST_SKIP();

  // cameraHeightM() ≈ 1.5 m
  const float h = estimator.cameraHeightM();
  EXPECT_NEAR(h, 1.5f, 0.1f)
      << "Altura estimada: " << h;
}

TEST(GroundEstimator, ReutilizaPlanoCacheadoEnFrameEstable) {
  GE::Config cfg;
  cfg.enable_plane_cache = true;
  GE estimator(cfg);
  const auto cloud = makeFlatCloud(800, 1.5f, 0.003f);

  ASSERT_TRUE(estimator.estimate(cloud));
  EXPECT_FALSE(estimator.perfStats().used_cached_plane);
  EXPECT_GT(estimator.perfStats().ransac_iterations, 0);

  ASSERT_TRUE(estimator.estimate(cloud));
  EXPECT_TRUE(estimator.perfStats().used_cached_plane);
  EXPECT_EQ(estimator.perfStats().ransac_iterations, 0);
  EXPECT_GT(static_cast<int>(estimator.groundIndices().size()), 100);
}
