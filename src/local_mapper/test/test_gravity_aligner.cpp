#include <gtest/gtest.h>
#include "local_mapper/gravity_aligner.hpp"
#include <cmath>

// ─────────────────────────────────────────────────────────────────────────────
// Tests de GravityAligner
//
// Convención del frame camera_depth_optical_frame:
//   X = derecha, Y = abajo, Z = adelante
// Cuando la cámara está horizontal, el acelerómetro mide (0, -g, 0).
// El vector "abajo" canónico es b = (0, -1, 0).
// La rotación calculada debe ser identidad cuando a = (0, -g, 0).
// ─────────────────────────────────────────────────────────────────────────────

TEST(GravityAligner, QuaternionIdentityNoRotation) {
  nav_math::Quaternion q{1.0f, 0.0f, 0.0f, 0.0f};
  const auto r = q.rotate(nav_math::Vec3{1.0f, 2.0f, 3.0f});
  EXPECT_NEAR(r.x, 1.0f, 1e-4f);
  EXPECT_NEAR(r.y, 2.0f, 1e-4f);
  EXPECT_NEAR(r.z, 3.0f, 1e-4f);
}

TEST(GravityAligner, EstimateOrientationFlat_IsIdentity) {
  // Cámara perfectamente horizontal → accel = (0, -g, 0) → rotación identidad
  local_mapper::GravityAligner aligner;
  const auto q = aligner.estimateOrientation(0.0f, -9.81f, 0.0f);
  EXPECT_NEAR(q.w, 1.0f, 0.05f);
  EXPECT_NEAR(q.x, 0.0f, 0.05f);
  EXPECT_NEAR(q.y, 0.0f, 0.05f);
  EXPECT_NEAR(q.z, 0.0f, 0.05f);
}

TEST(GravityAligner, AlignToGravity_FlatPreservesCoords) {
  // Con la cámara horizontal la alineación no debe modificar los puntos
  local_mapper::GravityAligner aligner;
  const auto pt = aligner.alignToGravity(1.0f, 2.0f, 3.0f, 0.0f, -9.81f, 0.0f);
  EXPECT_NEAR(pt[0], 1.0f, 0.05f);
  EXPECT_NEAR(pt[1], 2.0f, 0.05f);
  EXPECT_NEAR(pt[2], 3.0f, 0.05f);
}

TEST(GravityAligner, AlignToGravity_TiltedGivesFiniteResult) {
  // Cámara mirando hacia abajo: gravedad apunta en +Z en el frame óptico
  local_mapper::GravityAligner aligner;
  const auto pt = aligner.alignToGravity(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 9.81f);
  EXPECT_TRUE(std::isfinite(pt[0]));
  EXPECT_TRUE(std::isfinite(pt[1]));
  EXPECT_TRUE(std::isfinite(pt[2]));
}

TEST(GravityAligner, EstimateOrientation_ZeroAccelReturnsIdentity) {
  // Aceleración nula no debe causar NaN ni división por cero
  local_mapper::GravityAligner aligner;
  const auto q = aligner.estimateOrientation(0.0f, 0.0f, 0.0f);
  EXPECT_TRUE(std::isfinite(q.w));
  EXPECT_TRUE(std::isfinite(q.x));
  EXPECT_TRUE(std::isfinite(q.y));
  EXPECT_TRUE(std::isfinite(q.z));
  // Debe retornar identidad
  EXPECT_NEAR(q.w, 1.0f, 1e-4f);
}

TEST(GravityAligner, QuaternionNormIsPreserved) {
  // La rotación no debe cambiar la norma del vector
  local_mapper::GravityAligner aligner;
  // Inclinación de 30° (roll): ax = sin(30°)·g ≈ 4.9, ay = -cos(30°)·g ≈ -8.5
  const auto pt = aligner.alignToGravity(1.0f, 0.0f, 0.0f, 4.905f, -8.495f, 0.0f);
  const float norm = std::sqrt(pt[0]*pt[0] + pt[1]*pt[1] + pt[2]*pt[2]);
  EXPECT_NEAR(norm, 1.0f, 1e-3f);
}
