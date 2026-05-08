// ─────────────────────────────────────────────────────────────────────────────
// test_complementary_filter.cpp
//
// Tests del filtro complementario de Mahony.
// ─────────────────────────────────────────────────────────────────────────────

#include <gtest/gtest.h>
#include <cmath>
#include "nav_odometry/complementary_filter.hpp"

using namespace nav_odometry;

// Construye un acelerómetro "perfecto" para la orientación q dada.
// Si la cámara está orientada por q (body->world), la gravedad en body frame es:
//   g_body = q^{-1} * g_world  donde g_world = {0, g, 0} (Y-abajo)
static Vec3 simulatedAccel(const Quaternion& q, float g = 9.807f) {
    const Vec3 g_world{0.f, g, 0.f};
    return q.conjugate().rotate(g_world);
}

// ─── Convergencia de tilt ─────────────────────────────────────────────────────

TEST(ComplementaryFilter, InitialOrientationIsIdentity) {
    ComplementaryFilter f;
    const Quaternion q = f.orientation();
    EXPECT_NEAR(q.w, 1.f, 1e-5f);
}

TEST(ComplementaryFilter, ConvergesWithStaticAccelNoGyro) {
    // Sin giroscopio y con acelerómetro perfecto --> debe mantenerse en identity
    ComplementaryFilter f;
    const double dt = 0.005;
    double t = 0.0;
    const Vec3 gyro_zero{0.f, 0.f, 0.f};

    for (int i = 0; i < 200; ++i) {
        const Vec3 a = simulatedAccel(f.orientation());
        f.processImu(t, gyro_zero, a);
        t += dt;
    }

    const Quaternion q = f.orientation();
    EXPECT_NEAR(q.w, 1.f, 0.02f);  // orientación debe permanecer cercana a identity
}

TEST(ComplementaryFilter, CorrectsTiltFromStaticAccel) {
    // Iniciar con orientación incorrecta (30 deg de error en pitch) y verificar
    // que el filtro converge al cabo de ~5 segundos.
    ComplementaryFilter::Config cfg;
    cfg.kp_accel = 2.0f;
    cfg.ki_accel = 0.005f;
    ComplementaryFilter f(cfg);

    // Perturbación inicial: 30 deg alrededor de X
    const Quaternion q_initial = Quaternion::fromAngleAxis(
        static_cast<float>(M_PI) / 6.f, Vec3{1.f, 0.f, 0.f});
    f.resetOrientation(q_initial);

    const double dt = 0.005;
    double t = 0.0;
    const Vec3 gyro_zero{0.f, 0.f, 0.f};

    // El acelerómetro siempre reporta la gravedad del frame del mundo
    const Vec3 a_world_body{0.f, 9.807f, 0.f};  // cámara horizontal

    for (int i = 0; i < 1200; ++i) {  // 6 segundos
        f.processImu(t, gyro_zero, a_world_body);
        t += dt;
    }

    const Quaternion q = f.orientation();
    // El eje X del error debe haberse reducido: q.x debe estar cerca de 0
    EXPECT_NEAR(q.w, 1.f, 0.05f);
    EXPECT_NEAR(q.x, 0.f, 0.05f);
}

TEST(ComplementaryFilter, GyroIntegratesYaw) {
    // 1 rad/s alrededor de Y sin corrección visual --> yaw debe acumularse
    ComplementaryFilter::Config cfg;
    cfg.kp_accel = 0.f;  // desactivar corrección de accel para aislar el gyro
    cfg.ki_accel = 0.f;
    ComplementaryFilter f(cfg);

    const double dt   = 0.005;
    const float  wy   = 1.f;  // rad/s
    const int    steps = static_cast<int>(M_PI / 2.0 / wy / dt);  // 90 deg

    double t = 0.0;
    for (int i = 0; i < steps; ++i) {
        f.processImu(t, Vec3{0.f, wy, 0.f}, Vec3{0.f, 9.807f, 0.f});
        t += dt;
    }

    const float yaw = f.orientation().yawY();
    EXPECT_NEAR(yaw, static_cast<float>(M_PI / 2.0), 0.02f);
}

TEST(ComplementaryFilter, VisualUpdateCorrectYaw) {
    // Simular error de yaw de 20 deg y aplicar corrección visual perfecta
    ComplementaryFilter::Config cfg;
    cfg.alpha_yaw = 0.8f;  // ganancia alta para test
    cfg.min_visual_confidence = 0.1f;
    ComplementaryFilter f(cfg);

    // Error inicial de yaw: 20 deg alrededor de Y
    const float yaw_err = static_cast<float>(M_PI) / 9.f;  // 20 deg
    const Quaternion q_with_error = Quaternion::fromAngleAxis(
        yaw_err, Vec3{0.f, 1.f, 0.f});
    f.resetOrientation(q_with_error);

    const float yaw_before = f.orientation().yawY();
    EXPECT_NEAR(yaw_before, yaw_err, 1e-4f);

    // Corrección visual perfecta: dR = identity (la cámara no rotó respecto a q_at_capture=identity)
    const Quaternion q_at_capture = Quaternion::identity();
    const Quaternion dR_body      = Quaternion::identity();
    f.applyVisualUpdate(dR_body, q_at_capture, 1.0f);

    const float yaw_after = f.orientation().yawY();
    // Con alpha_yaw=0.8 y confianza=1.0, el yaw debe haberse reducido notablemente
    EXPECT_LT(std::abs(yaw_after), std::abs(yaw_before));
}

TEST(ComplementaryFilter, LowConfidenceVisualIgnored) {
    // Si la confianza es menor que min_visual_confidence, no debe haber corrección
    ComplementaryFilter::Config cfg;
    cfg.min_visual_confidence = 0.5f;
    ComplementaryFilter f(cfg);

    const float yaw_err = 0.3f;
    f.resetOrientation(Quaternion::fromAngleAxis(yaw_err, Vec3{0.f, 1.f, 0.f}));

    const float yaw_before = f.orientation().yawY();
    f.applyVisualUpdate(Quaternion::identity(), Quaternion::identity(), 0.1f);
    const float yaw_after = f.orientation().yawY();

    // No debe haber corrección
    EXPECT_NEAR(yaw_after, yaw_before, 1e-4f);
}

TEST(ComplementaryFilter, NormPreservedAfterUpdates) {
    ComplementaryFilter f;
    double t = 0.0;
    for (int i = 0; i < 500; ++i) {
        f.processImu(t, Vec3{0.1f, 0.3f, -0.2f}, Vec3{0.5f, 9.5f, 0.3f});
        if (i % 33 == 0) {
            f.applyVisualUpdate(
                Quaternion::fromAngleAxis(0.05f, Vec3{0.f, 1.f, 0.f}),
                f.orientation(), 0.8f);
        }
        t += 0.005;
    }
    const float norm = std::sqrt(f.orientation().norm2());
    EXPECT_NEAR(norm, 1.f, 1e-4f);
}

TEST(ComplementaryFilter, Reset) {
    ComplementaryFilter f;
    double t = 0.0;
    for (int i = 0; i < 100; ++i) {
        f.processImu(t, Vec3{1.f, 1.f, 1.f}, Vec3{0.f, 9.8f, 0.f});
        t += 0.005;
    }
    f.reset();
    const Quaternion q = f.orientation();
    EXPECT_NEAR(q.w, 1.f, 1e-5f);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
