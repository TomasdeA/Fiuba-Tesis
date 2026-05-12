// ─────────────────────────────────────────────────────────────────────────────
// nav_odometry/rgbd_tracker.cpp
//
// Odometría visual: FAST --> Lucas-Kanade temporal --> PnP + RANSAC.
//
// Estrategia de robustez:
//   - Features distribuidos en grilla para evitar agrupamiento.
//   - Profundidad: depth image de hardware (setDepthFrame antes de process()).
//   - RANSAC en solvePnPRansac filtra outliers de movimiento.
//   - Resultado inválido si inliers < umbral (tracking failure).
// ─────────────────────────────────────────────────────────────────────────────

#include "nav_odometry/rgbd_tracker.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/video/tracking.hpp>

#include <cmath>
#include <algorithm>
#include <numeric>

namespace nav_odometry {

RgbdTracker::RgbdTracker()
    : RgbdTracker(Config{}) {}

RgbdTracker::RgbdTracker(const Config& cfg)
    : cfg_(cfg) {}

void RgbdTracker::setCameraIntrinsics(const CameraIntrinsics& intrinsics) {
    intrinsics_ = intrinsics;
    if (intrinsics_.valid) {
        K_ = (cv::Mat_<double>(3, 3) <<
            intrinsics_.fx, 0.0,            intrinsics_.cx,
            0.0,            intrinsics_.fy, intrinsics_.cy,
            0.0,            0.0,            1.0);
    }
    initialized_ = false;
}

void RgbdTracker::setDepthFrame(const DepthFrame& frame) noexcept {
    depth_frame_ = frame;
}

void RgbdTracker::reset() {
    initialized_ = false;
    prev_pts_.clear();
    prev_pts3d_.clear();
    prev_frame_ = cv::Mat{};
}

VisualOdometryResult RgbdTracker::process(const ColorFrame& frame)
{
    VisualOdometryResult result;

    if (!intrinsics_.valid) return result;

    // Convertir a cv::Mat sin copia de datos (vista sobre buffer ROS2)
    const cv::Mat curr_frame(
        static_cast<int>(frame.height),
        static_cast<int>(frame.width),
        CV_8UC1,
        const_cast<uint8_t*>(frame.data),
        frame.step);

    // ── Primer frame: inicialización (Etapas 1+2) ─────────────────────────────
    // Se detectan esquinas (Etapa 1) y se levanta a 3D (Etapa 2) para
    // establecer el primer fotograma de referencia.
    if (!initialized_) {
        curr_frame.copyTo(prev_frame_);
        detectFeatures(prev_frame_, prev_pts_);          // Etapa 1
        prev_pts3d_ = liftFromDepth(prev_pts_, prev_pts_); // Etapa 2
        initialized_ = !prev_pts_.empty();
        return result;
    }

    // ── Etapa 3: seguimiento Lucas-Kanade piramidal ──────────────────────────────
    // prev_frame_ / prev_pts_ / prev_pts3d_ contienen el fotograma de referencia
    // con sus puntos detectados (Etapa 1) y levantados a 3D (Etapa 2).
    // Esos datos se cargan durante la inicialización o al final de cada ciclo
    // exitoso (ver actualización de estado al final de esta función).
    std::vector<cv::Point2f> curr_pts;
    std::vector<uchar>       lk_status;
    std::vector<float>       lk_err;

    // Guard: si prev_pts_ está vacío el assert de LK falla → reset (Etapas 1+2)
    if (prev_pts_.empty()) {
        curr_frame.copyTo(prev_frame_);
        detectFeatures(prev_frame_, prev_pts_);          // Etapa 1
        prev_pts3d_ = liftFromDepth(prev_pts_, prev_pts_); // Etapa 2
        initialized_ = !prev_pts_.empty();
        return result;
    }

    // prev_frame_  : imagen de referencia (t)
    // curr_frame : imagen actual (t+1)
    // prev_pts_   : puntos en la imagen anterior que se quiere seguir
    // curr_pts    : (salida) nueva posición estimada de esos puntos en la imagen actual
    // lk_status   : (salida) vector de flags (1 = tracking exitoso, 0 = punto perdido)
    // lk_err      : (salida) error de tracking por punto (menor = mejor match)
    // lk_win      : tamaño de la ventana (parche) usada para buscar el punto (ej: 21x21)
    // cfg_.lk_pyramid_levels : niveles de pirámide (permite manejar movimientos grandes)
    const cv::Size lk_win(cfg_.lk_window_size, cfg_.lk_window_size);
    cv::calcOpticalFlowPyrLK(
        prev_frame_, curr_frame,
        prev_pts_, curr_pts,
        lk_status, lk_err,
        lk_win, cfg_.lk_pyramid_levels);

    // Filtrar puntos tracked con éxito
    std::vector<cv::Point2f>  pts_prev_ok, pts_curr_ok;
    std::vector<cv::Point3f>  pts3d_ok;

    for (std::size_t i = 0; i < lk_status.size(); ++i) {
        if (lk_status[i] && i < prev_pts3d_.size()) {
            pts_prev_ok.push_back(prev_pts_[i]);
            pts_curr_ok.push_back(curr_pts[i]);
            pts3d_ok.push_back(prev_pts3d_[i]);
        }
    }
    last_num_tracked_ = static_cast<int>(pts_curr_ok.size());

    if (last_num_tracked_ < cfg_.min_inliers) {
        // Tracking failure: reiniciar fotograma de referencia (Etapas 1+2)
        curr_frame.copyTo(prev_frame_);
        detectFeatures(prev_frame_, prev_pts_);          // Etapa 1
        prev_pts3d_ = liftFromDepth(prev_pts_, prev_pts_); // Etapa 2
        initialized_ = !prev_pts_.empty();
        return result;
    }

    // ── Etapa 4: estimación de pose 6-DOF mediante PnP con RANSAC ─────────────
    // Ordenar por profundidad ascendente: los puntos más cercanos tienen menor
    // ruido (sigma_z ~ z^2) y se priorizan para el refinamiento ponderado, que
    // los replica proporcionalmente para asignarles mayor influencia en la pose.
    {
        std::vector<std::size_t> order(pts3d_ok.size());
        std::iota(order.begin(), order.end(), std::size_t{0});
        std::sort(order.begin(), order.end(),
                  [&](std::size_t a, std::size_t b){
                      return pts3d_ok[a].z < pts3d_ok[b].z;
                  });
        std::vector<cv::Point3f> tmp3d(pts3d_ok.size());
        std::vector<cv::Point2f> tmp2d(pts_curr_ok.size());
        for (std::size_t i = 0; i < order.size(); ++i) {
            tmp3d[i] = pts3d_ok   [order[i]];
            tmp2d[i] = pts_curr_ok[order[i]];
        }
        pts3d_ok    = std::move(tmp3d);
        pts_curr_ok = std::move(tmp2d);
    }

    // EPnP + RANSAC: puntos 3D de referencia {P_i} → proyecciones 2D del frame actual {x_i}
    cv::Mat rvec, tvec;
    std::vector<int> pnp_inliers;

    const bool pnp_ok = cv::solvePnPRansac(
        pts3d_ok,       // puntos 3D en el marco de referencia (t-1)
        pts_curr_ok,    // sus proyecciones 2D en el frame actual (t)
        K_,             // matriz intrínseca de la cámara
        cv::noArray(),  // sin coeficientes de distorsión (imagen ya rectificada por RealSense)
        rvec, tvec,     // salida: rotación (Rodrigues) y traslación
        false,          // no usar estimación inicial (RANSAC empieza desde cero)
        100,            // número de iteraciones RANSAC
        static_cast<float>(cfg_.essential_ransac_threshold),  // umbral de reproyección (px)
        0.99,           // confianza deseada (probabilidad de que alguna iteración sea libre de outliers)
        pnp_inliers,    // salida: índices de los inliers del consenso ganador
        cv::SOLVEPNP_EPNP);  // método: EPnP, complejidad O(N)

    last_num_inliers_ = static_cast<int>(pnp_inliers.size());

    const float inlier_ratio = (last_num_tracked_ > 0)
        ? static_cast<float>(last_num_inliers_) / static_cast<float>(last_num_tracked_)
        : 0.f;
    // Condiciones de fallo: error interno de OpenCV, pocos inliers absolutos, o ratio demasiado
    // bajo.
    if (!pnp_ok
        || last_num_inliers_ < cfg_.min_inliers
        || inlier_ratio < cfg_.min_inlier_ratio)
    {
        // PnP no confiable: reiniciar fotograma de referencia (Etapas 1+2)
        curr_frame.copyTo(prev_frame_);
        detectFeatures(prev_frame_, prev_pts_);          // Etapa 1
        prev_pts3d_ = liftFromDepth(prev_pts_, prev_pts_); // Etapa 2
        return result;
    }

    // ── Refinamiento ponderado por profundidad (Levenberg-Marquardt) ───────────
    //
    // solvePnPRefineLM no acepta pesos explícitos. Para dar más influencia
    // a los puntos cercanos (ruido sigma_z ~ z^2 segun D435i), se replican
    // proporcionalmente a round(1/z_i):
    //   z = 0.25 m --> 4 réplicas   z = 0.5 m --> 2 réplicas   z >= 1.0 m --> 1 réplica
    // El ordenamiento previo por profundidad no afecta al resultado numérico
    // pero facilita la construcción del arreglo aumentado (los más replicados
    // quedan concentrados al inicio).
    {
        std::vector<cv::Point3f> pts3d_ref;
        std::vector<cv::Point2f> pts2d_ref;
        pts3d_ref.reserve(pnp_inliers.size() * 4);
        pts2d_ref.reserve(pnp_inliers.size() * 4);
        for (int i : pnp_inliers) {
            const int reps = std::max(1,
                static_cast<int>(std::round(1.0f / pts3d_ok[i].z)));
            for (int r = 0; r < reps; ++r) {
                pts3d_ref.push_back(pts3d_ok[i]);
                pts2d_ref.push_back(pts_curr_ok[i]);
            }
        }
        //
        cv::solvePnPRefineLM(pts3d_ref, pts2d_ref, K_, cv::noArray(), rvec, tvec);
    }

    // ── Verificación de magnitud de translación ───────────────────────────────
    const double tx = tvec.at<double>(0);
    const double ty = tvec.at<double>(1);
    const double tz = tvec.at<double>(2);
    const float t_mag = static_cast<float>(std::sqrt(tx*tx + ty*ty + tz*tz));

    if (t_mag > cfg_.max_translation_per_frame_m) {
        // Traslación no plausible (blur): reiniciar fotograma de referencia (Etapas 1+2)
        curr_frame.copyTo(prev_frame_);
        detectFeatures(prev_frame_, prev_pts_);          // Etapa 1
        prev_pts3d_ = liftFromDepth(prev_pts_, prev_pts_); // Etapa 2
        return result;
    }

    // ── Etapa 5: extracción del cuaternión y actualización del fotograma de referencia ─
    // PnP retorna la transformación CÁMARA_ACTUAL --> PUNTOS3D (i.e., pose inversa).
    // Para obtener dR y dT en body frame:
    //   el rvec de PnP lleva de world (t-1) a cámara (t)

    cv::Mat R_cv;
    cv::Rodrigues(rvec, R_cv);

    // Extraer quaternión de la matriz de rotación 3x3 (método de Shepperd)
    const double r00 = R_cv.at<double>(0,0), r01 = R_cv.at<double>(0,1), r02 = R_cv.at<double>(0,2);
    const double r10 = R_cv.at<double>(1,0), r11 = R_cv.at<double>(1,1), r12 = R_cv.at<double>(1,2);
    const double r20 = R_cv.at<double>(2,0), r21 = R_cv.at<double>(2,1), r22 = R_cv.at<double>(2,2);

    const double trace = r00 + r11 + r22;
    float qw, qx, qy, qz;

    if (trace > 0.0) {
        const double s = 0.5 / std::sqrt(trace + 1.0);
        qw = static_cast<float>(0.25 / s);
        qx = static_cast<float>((r21 - r12) * s);
        qy = static_cast<float>((r02 - r20) * s);
        qz = static_cast<float>((r10 - r01) * s);
    } else if (r00 > r11 && r00 > r22) {
        const double s = 2.0 * std::sqrt(1.0 + r00 - r11 - r22);
        qw = static_cast<float>((r21 - r12) / s);
        qx = static_cast<float>(0.25 * s);
        qy = static_cast<float>((r01 + r10) / s);
        qz = static_cast<float>((r02 + r20) / s);
    } else if (r11 > r22) {
        const double s = 2.0 * std::sqrt(1.0 + r11 - r00 - r22);
        qw = static_cast<float>((r02 - r20) / s);
        qx = static_cast<float>((r01 + r10) / s);
        qy = static_cast<float>(0.25 * s);
        qz = static_cast<float>((r12 + r21) / s);
    } else {
        const double s = 2.0 * std::sqrt(1.0 + r22 - r00 - r11);
        qw = static_cast<float>((r10 - r01) / s);
        qx = static_cast<float>((r02 + r20) / s);
        qy = static_cast<float>((r12 + r21) / s);
        qz = static_cast<float>(0.25 * s);
    }

    result.delta_rotation    = Quaternion{qw, qx, qy, qz}.normalized();
    // solvePnP devuelve t tal que P_cam_t = R·P_{t-1} + t  (origen de t-1 en frame t).
    // El desplazamiento de la cámara en frame t-1 es -R^T·t ≈ -t (rotación pequeña por frame).
    result.delta_translation = Vec3{static_cast<float>(-tx),
                                    static_cast<float>(-ty),
                                    static_cast<float>(-tz)};
    result.num_inliers = last_num_inliers_;
    result.confidence  = inlier_ratio;
    result.valid       = true;

    // ── Actualizar estado para el siguiente frame ─────────────────────────────
    curr_frame.copyTo(prev_frame_);

    // Repoblar puntos usando los inliers del PnP, y rellenar si hay pocos
    std::vector<cv::Point2f> new_pts;
    new_pts.reserve(pnp_inliers.size());
    for (int idx : pnp_inliers) {
        new_pts.push_back(pts_curr_ok[idx]);
    }

    // Si quedaron pocos puntos, añadir nuevos detectados en este frame
    if (static_cast<int>(new_pts.size()) < cfg_.max_features / 2) {
        std::vector<cv::Point2f> extra_pts;
        detectFeatures(curr_frame, extra_pts);
        // Filtrar extra_pts que estén demasiado cerca de los nuevos
        for (const auto& ep : extra_pts) {
            bool too_close = false;
            for (const auto& np : new_pts) {
                const float dx = ep.x - np.x;
                const float dy = ep.y - np.y;
                if (dx*dx + dy*dy < cfg_.min_feature_dist_px * cfg_.min_feature_dist_px) {
                    too_close = true;
                    break;
                }
            }
            if (!too_close) new_pts.push_back(ep);
            if (static_cast<int>(new_pts.size()) >= cfg_.max_features) break;
        }
    }

    // Etapas 1+2 para el siguiente ciclo: new_pts ya contiene los puntos
    // seleccionados (inliers PnP ± FAST de relleno); se levantan a 3D.
    prev_pts_   = new_pts;
    prev_pts3d_ = liftFromDepth(prev_pts_, prev_pts_); // Etapa 2

    return result;
}

void RgbdTracker::detectFeatures(const cv::Mat& img,
                                    std::vector<cv::Point2f>& pts)
{
    // ── Etapa 1: detección distribuida de esquinas FAST ───────────────────────
    pts.clear();

    // Detección FAST distribuida en una grilla para evitar agrupamiento
    const int cell_w = img.cols / 4;
    const int cell_h = img.rows / 3;
    const int max_per_cell = cfg_.max_features / 12 + 1;

    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 4; ++col) {
            const cv::Rect roi(col * cell_w, row * cell_h, cell_w, cell_h);
            cv::Mat cell = img(roi);
            std::vector<cv::KeyPoint> kps;
            cv::FAST(cell, kps, cfg_.fast_threshold, true);

            // Ordenar por respuesta (esquinas más fuertes primero)
            std::sort(kps.begin(), kps.end(),
                      [](const cv::KeyPoint& a, const cv::KeyPoint& b) {
                          return a.response > b.response;
                      });

            int n = 0;
            for (const auto& kp : kps) {
                if (n >= max_per_cell) break;
                pts.emplace_back(kp.pt.x + col * cell_w,
                                 kp.pt.y + row * cell_h);
                ++n;
            }
        }
    }
}

std::vector<cv::Point3f> RgbdTracker::liftFromDepth(
    const std::vector<cv::Point2f>& pts_in,
    std::vector<cv::Point2f>& pts_out)
{
    // Proyección inversa pinhole: (u, v, d) → P = ((u-cx)·d/fx, (v-cy)·d/fy, d).
    // Se descartan píxeles sin dato de profundidad (raw=0) o cuya profundidad
    // resultante quede fuera del rango válido [depth_min_m, depth_max_m].
    const float inv_fx  = 1.f / intrinsics_.fx;
    const float inv_fy  = 1.f / intrinsics_.fy;
    const float cx      = intrinsics_.cx;
    const float cy      = intrinsics_.cy;
    const int   dw      = static_cast<int>(depth_frame_.width);
    const int   dh      = static_cast<int>(depth_frame_.height);
    // step está en bytes; cada pixel es uint16_t (2 bytes)
    const int   stride  = static_cast<int>(depth_frame_.step) / 2;
    const float scale   = depth_frame_.scale_m;

    std::vector<cv::Point3f> pts3d;
    pts3d.reserve(pts_in.size());
    std::vector<cv::Point2f> pts_filtered;
    pts_filtered.reserve(pts_in.size());

    for (const auto& p : pts_in) {
        const int u = static_cast<int>(std::round(p.x));
        const int v = static_cast<int>(std::round(p.y));
        if (u < 0 || u >= dw || v < 0 || v >= dh) continue;
        const uint16_t raw = depth_frame_.data[v * stride + u];
        if (raw == 0) continue;
        const float Z = static_cast<float>(raw) * scale;
        if (Z < cfg_.depth_min_m || Z > cfg_.depth_max_m) continue;
        pts3d.emplace_back(
            (static_cast<float>(u) - cx) * Z * inv_fx,
            (static_cast<float>(v) - cy) * Z * inv_fy,
            Z);
        pts_filtered.push_back(p);
    }

    pts_out = pts_filtered;
    return pts3d;
}

}  // namespace nav_odometry
