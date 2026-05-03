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
    prev_left_ = cv::Mat{};
}

VisualOdometryResult RgbdTracker::process(const ColorFrame& frame)
{
    VisualOdometryResult result;

    if (!intrinsics_.valid) return result;

    // Convertir a cv::Mat sin copia de datos (vista sobre buffer ROS2)
    const cv::Mat left_img(
        static_cast<int>(frame.height),
        static_cast<int>(frame.width),
        CV_8UC1,
        const_cast<uint8_t*>(frame.data),
        frame.step);

    // ── Primer frame: inicialización ─────────────────────────────────────────
    if (!initialized_) {
        left_img.copyTo(prev_left_);
        detectFeatures(prev_left_, prev_pts_);
        prev_pts3d_ = liftFromDepth(prev_pts_, prev_pts_);
        initialized_ = !prev_pts_.empty();
        return result;
    }

    // ── Tracking Lucas-Kanade: frame anterior --> frame actual ─────────────────
    std::vector<cv::Point2f> curr_pts;
    std::vector<uchar>       lk_status;
    std::vector<float>       lk_err;

    // Guard: si prev_pts_ está vacío el assert de LK falla
    if (prev_pts_.empty()) {
        left_img.copyTo(prev_left_);
        detectFeatures(prev_left_, prev_pts_);
        prev_pts3d_ = liftFromDepth(prev_pts_, prev_pts_);
        initialized_ = !prev_pts_.empty();
        return result;
    }

    const cv::Size lk_win(cfg_.lk_window_size, cfg_.lk_window_size);
    cv::calcOpticalFlowPyrLK(
        prev_left_, left_img,
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
        // Tracking failure: reiniciar con este frame como nuevo keyframe
        left_img.copyTo(prev_left_);
        detectFeatures(prev_left_, prev_pts_);
        prev_pts3d_ = liftFromDepth(prev_pts_, prev_pts_);
        initialized_ = !prev_pts_.empty();
        return result;
    }

    // ── Ordenar por profundidad ascendente ────────────────────────────────────
    // Puntos más cercanos tienen menor ruido de profundidad (sigma_z ~ z^2), por lo
    // que se priorizan colocándolos primero en el arreglo.  El posterior paso de
    // refinamiento ponderado los replica proporcionalmente para asignarles mayor
    // influencia en la estimación de pose.
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

    // ── Estimación de pose 3D-2D con PnP + RANSAC ────────────────────────────
    // Usamos los puntos 3D del frame anterior y sus proyecciones en el frame actual.
    cv::Mat rvec, tvec;
    std::vector<int> pnp_inliers;

    const bool pnp_ok = cv::solvePnPRansac(
        pts3d_ok,
        pts_curr_ok,
        K_,
        cv::noArray(),   // sin distorsión (imagen ya rectificada)
        rvec, tvec,
        false,           // no usar estimación inicial
        100,             // iteraciones RANSAC
        static_cast<float>(cfg_.essential_ransac_threshold),
        0.99,            // confianza RANSAC
        pnp_inliers,
        cv::SOLVEPNP_EPNP);

    last_num_inliers_ = static_cast<int>(pnp_inliers.size());

    const float inlier_ratio = (last_num_tracked_ > 0)
        ? static_cast<float>(last_num_inliers_) / static_cast<float>(last_num_tracked_)
        : 0.f;

    if (!pnp_ok
        || last_num_inliers_ < cfg_.min_inliers
        || inlier_ratio < cfg_.min_inlier_ratio)
    {
        // Resultado de PnP no confiable: usar frame actual como referencia
        left_img.copyTo(prev_left_);
        detectFeatures(prev_left_, prev_pts_);
        prev_pts3d_ = liftFromDepth(prev_pts_, prev_pts_);
        return result;
    }

    // ── Refinamiento ponderado por profundidad ────────────────────────────────
    // El ruido del D435i crece con z^2; los puntos más cercanos son más fiables.
    // Se simulan pesos w_i = round(1 / z_i) mediante replicación de inliers
    // antes de un paso de refinamiento Levenberg-Marquardt.
    //   z = 0.25 m --> 4 réplicas   z = 0.5 m --> 2 réplicas   z >= 1.0 m --> 1 réplica
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
        cv::solvePnPRefineLM(pts3d_ref, pts2d_ref, K_, cv::noArray(), rvec, tvec);
    }

    // ── Verificación de magnitud de translación ───────────────────────────────
    const double tx = tvec.at<double>(0);
    const double ty = tvec.at<double>(1);
    const double tz = tvec.at<double>(2);
    const float t_mag = static_cast<float>(std::sqrt(tx*tx + ty*ty + tz*tz));

    if (t_mag > cfg_.max_translation_per_frame_m) {
        // Movimiento demasiado grande para ser creíble; probablemente blur
        left_img.copyTo(prev_left_);
        detectFeatures(prev_left_, prev_pts_);
        prev_pts3d_ = liftFromDepth(prev_pts_, prev_pts_);
        return result;
    }

    // ── Convertir rvec / tvec a tipos internos ────────────────────────────────
    // PnP retorna la transformación CÁMARA_ACTUAL --> PUNTOS3D (i.e., pose inversa).
    // Para obtener dR y dT en body frame:
    //   el rvec de PnP lleva de world (t-1) a cámara (t), que es lo que queremos.

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
    result.delta_translation = Vec3{static_cast<float>(tx),
                                    static_cast<float>(ty),
                                    static_cast<float>(tz)};
    result.num_inliers = last_num_inliers_;
    result.confidence  = inlier_ratio;
    result.valid       = true;

    // ── Actualizar estado para el siguiente frame ─────────────────────────────
    left_img.copyTo(prev_left_);

    // Repoblar puntos usando los inliers del PnP, y rellenar si hay pocos
    std::vector<cv::Point2f> new_pts;
    new_pts.reserve(pnp_inliers.size());
    for (int idx : pnp_inliers) {
        new_pts.push_back(pts_curr_ok[idx]);
    }

    // Si quedaron pocos puntos, añadir nuevos detectados en este frame
    if (static_cast<int>(new_pts.size()) < cfg_.max_features / 2) {
        std::vector<cv::Point2f> extra_pts;
        detectFeatures(left_img, extra_pts);
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

    prev_pts_  = new_pts;
    prev_pts3d_ = liftFromDepth(prev_pts_, prev_pts_);

    return result;
}

void RgbdTracker::detectFeatures(const cv::Mat& img,
                                    std::vector<cv::Point2f>& pts)
{
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
