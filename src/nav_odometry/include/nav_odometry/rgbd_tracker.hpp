#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// nav_odometry/rgbd_tracker.hpp
//
// Odometría visual usando flujo óptico Lucas-Kanade temporal + PnP con
// profundidad de hardware.
//
// Pipeline:
//   1. Detección FAST de esquinas en imagen color (mono 8-bit).
//   2. Tracking Lucas-Kanade entre frame t-1 y frame t.
//   3. Lifting de puntos 2D->3D desde el depth image del sensor.
//   4. PnP (solvePnPRansac) para estimar pose 6-DOF relativa.
//
// Requiere OpenCV 4.x.
// ─────────────────────────────────────────────────────────────────────────────

#include "nav_odometry/types.hpp"

#include <opencv2/core.hpp>
#include <vector>

namespace nav_odometry {

class RgbdTracker {
public:
    // ─── Configuración ────────────────────────────────────────────────────────
    struct Config {
        // Detector FAST: umbral para la puntuación de esquina.
        int fast_threshold{20};
        // Número máximo de puntos rastreados por frame.
        int max_features{300};
        // Distancia mínima entre puntos detectados (px) para distribuirlos.
        float min_feature_dist_px{15.f};
        // Umbral de error de reproyección para RANSAC del PnP [px].
        double essential_ransac_threshold{1.0};
        // Confianza mínima de inliers para aceptar el resultado de VO.
        // Si inliers/total < min_inlier_ratio, el resultado se marca como inválido.
        float min_inlier_ratio{0.5f};
        // Número mínimo absoluto de inliers para aceptar el resultado.
        int min_inliers{20};
        // Máximo desplazamiento de translación permitido por frame [m].
        // Rechaza estimaciones espurias debidas a blur o movimiento rápido.
        float max_translation_per_frame_m{0.5f};
        // Niveles de pirámide para Lucas-Kanade temporal.
        int lk_pyramid_levels{3};
        // Tamaño de ventana para Lucas-Kanade (píxeles, lado del cuadrado).
        int lk_window_size{21};
        // Rango de profundidad válido [m].
        // Píxeles cuya profundidad proyectada caiga fuera de este intervalo se
        // descartan antes de cualquier estimación de pose.
        float depth_min_m{0.25f};
        float depth_max_m{5.0f};
    };

    RgbdTracker();
    explicit RgbdTracker(const Config& cfg);

    // Configura la intrínseca antes de procesar frames.
    void setCameraIntrinsics(const CameraIntrinsics& intrinsics);

    // Procesa un frame visual y retorna la pose relativa respecto al frame anterior.
    // Requiere que setDepthFrame() se haya llamado con el depth del mismo instante.
    // Retorna VisualOdometryResult::valid = false en el primer frame o si el
    // tracking falla.
    VisualOdometryResult process(const ColorFrame& frame);

    // Fuerza la inicialización del frame siguiente (descarta el estado actual).
    void reset();

    // Actualiza el depth image para el próximo process().
    // No copia datos; el buffer debe permanecer válido durante process().
    void setDepthFrame(const DepthFrame& frame) noexcept;

    bool hasIntrinsics() const noexcept { return intrinsics_.valid; }
    bool isInitialized() const noexcept { return initialized_; }

    // Estadísticas del último frame procesado (para benchmarking).
    int lastNumTracked()  const noexcept { return last_num_tracked_; }
    int lastNumInliers()  const noexcept { return last_num_inliers_; }

private:
    Config           cfg_;
    CameraIntrinsics intrinsics_;
    bool             initialized_{false};

    // Estado del fotograma de referencia
    cv::Mat                  prev_frame_;
    std::vector<cv::Point2f> prev_pts_;    // puntos detectados en el fotograma de referencia (t-1)
    std::vector<cv::Point3f> prev_pts3d_;  // puntos 3D del fotograma de referencia (t-1)

    // Estadísticas
    int last_num_tracked_{0};
    int last_num_inliers_{0};

    // Depth image más reciente
    DepthFrame depth_frame_{};

    cv::Mat K_;   // matriz intrínseca

    // Detecta nuevos puntos FAST distribuyéndolos en una grilla.
    void detectFeatures(const cv::Mat& img, std::vector<cv::Point2f>& pts);

    // Lifting de puntos 2D a 3D desde el depth image.
    // Descarta puntos fuera de rango o con depth == 0.
    std::vector<cv::Point3f> liftFromDepth(
        const std::vector<cv::Point2f>& pts_in,
        std::vector<cv::Point2f>& pts_out);
};

}  // namespace nav_odometry
