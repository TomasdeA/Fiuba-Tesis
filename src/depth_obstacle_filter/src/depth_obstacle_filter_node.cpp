// ─────────────────────────────────────────────────────────────────────────────
// DepthObstacleFilterNode
//
// Suscribe:  depth/image  (sensor_msgs/Image, 16UC1)
//            depth/camera_info (sensor_msgs/CameraInfo)
//            nav_odom     (nav_msgs/Odometry) — orientación VIO de nav_odometry
//
// Publica:
//   /depth_obstacle_filter/obstacle_cloud  — obstáculos locales en gravity_aligned_frame
//                                            para obstacle_grid_encoder/heatmap
//
// Publica sólo con publish_local_mapper_interface=true:
//   /depth_obstacle_filter/obstacle_cloud_odom — obstáculos en frame odom
//   /depth_obstacle_filter/free_endpoints  — endpoints de rayos libres en frame odom
//   /depth_obstacle_filter/sensor_pos      — posición de la cámara en odom (PointStamped)
//                                            point.y = altura estimada del suelo (m)
//
// Publica sólo con debug=true:
//   /depth_obstacle_filter/debug/depth_cloud   — nube cruda en camera_depth_optical_frame
//   /depth_obstacle_filter/debug/raw_cloud     — nube cruda en gravity_aligned_frame
//   /depth_obstacle_filter/debug/ground_cloud  — suelo en gravity_aligned_frame
//   /depth_obstacle_filter/debug/ceiling_cloud — techo descartado en gravity_aligned_frame
//
// Publica señal liviana de calibración:
//   /depth_obstacle_filter/camera_height       — altura de cámara sobre el suelo (Float32)
//
// Publica TFs:
//   odom → gravity_aligned_frame          (yaw + traslación desde nav_odom)
//   gravity_aligned_frame → camera_depth_optical_frame  (roll + pitch)
//
// Los tres topics de interface (obstacle_cloud_odom, free_endpoints, sensor_pos)
// comparten siempre el mismo header.stamp (= stamp del frame de profundidad).
// local_mapper usa ExactTimeSynchronizer sobre los tres para garantizar
// sincronización perfecta: cada actualización del mapa usa exactamente
// los obstáculos y la posición del sensor del mismo instante.
// ─────────────────────────────────────────────────────────────────────────────

#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <time.h>

#include "depth_obstacle_filter/depth_projector.hpp"
#include "depth_obstacle_filter/gravity_aligner.hpp"
#include "depth_obstacle_filter/imu_filter.hpp"
#include "depth_obstacle_filter/ground_estimator.hpp"

using std::placeholders::_1;

class DepthObstacleFilterNode : public rclcpp::Node {
 public:
  DepthObstacleFilterNode() : rclcpp::Node("depth_obstacle_filter_node") {
    // ── Parámetros ───────────────────────────────────────────────────────────
    range_min_m_ = static_cast<float>(
        declare_parameter<double>("range_min_m", 0.1));
    range_max_m_ = static_cast<float>(
        declare_parameter<double>("range_max_m", 5.0));
    depth_pixel_stride_ = std::max(
        1, static_cast<int>(declare_parameter<int>("depth_pixel_stride", 1)));

    depth_obstacle_filter::GroundEstimator::Config ge_cfg;
    ge_cfg.enable_voxel_filter =
        declare_parameter<bool>("enable_voxel_filter", true);
    ge_cfg.voxel_size_m   = static_cast<float>(
        declare_parameter<double>("voxel_size_m", 0.05));
    ge_cfg.ransac_max_iter = declare_parameter<int>("ransac_max_iter", 100);
    ge_cfg.ransac_max_band_points =
        declare_parameter<int>("ransac_max_band_points", 5000);
    ge_cfg.ransac_inlier_tol = static_cast<float>(
        declare_parameter<double>("ground_inlier_tol", 0.0));
    ge_cfg.min_person_height_m = static_cast<float>(
        declare_parameter<double>("min_person_height_m", 1.3));
    ge_cfg.ceiling_delta_m = static_cast<float>(
        declare_parameter<double>("ceiling_delta_m", 0.1));
    ge_cfg.enable_plane_cache =
        declare_parameter<bool>("enable_plane_cache", true);
    ge_cfg.cached_plane_min_quality = static_cast<float>(
        declare_parameter<double>("cached_plane_min_quality", 0.45));
    ge_cfg.cached_plane_min_inliers =
        declare_parameter<int>("cached_plane_min_inliers", 80);
    ground_estimator_ = std::make_unique<depth_obstacle_filter::GroundEstimator>(ge_cfg);
    perf_log_enabled_ = declare_parameter<bool>("perf_log_enabled", false);
    perf_log_period_s_ = declare_parameter<double>("perf_log_period_s", 5.0);
    empty_obstacle_warn_every_ = declare_parameter<int>("empty_obstacle_warn_every", 5);
    debug_enabled_ = declare_parameter<bool>("debug", false);
    publish_local_mapper_interface_ =
        declare_parameter<bool>("publish_local_mapper_interface", false);
    orientation_source_ =
        declare_parameter<std::string>("orientation_source", "nav_odom");
    if (orientation_source_ != "nav_odom" &&
        orientation_source_ != "imu_legacy") {
      RCLCPP_WARN(get_logger(),
          "orientation_source='%s' invalido; usando nav_odom",
          orientation_source_.c_str());
      orientation_source_ = "nav_odom";
    }
    height_filter_alpha_ = static_cast<float>(
        declare_parameter<double>("height_filter_alpha", 0.03));
    height_max_step_m_ = static_cast<float>(
        declare_parameter<double>("height_max_step_m", 0.005));
    height_outlier_reject_m_ = static_cast<float>(
        declare_parameter<double>("height_outlier_reject_m", 0.25));
    height_publish_delta_m_ = static_cast<float>(
        declare_parameter<double>("height_publish_delta_m", 0.05));
    height_min_ground_quality_ = static_cast<float>(
        declare_parameter<double>("height_min_ground_quality", 0.35));
    height_min_ground_inliers_ =
        declare_parameter<int>("height_min_ground_inliers", 50);
    height_max_ground_tilt_deg_ = static_cast<float>(
        declare_parameter<double>("height_max_ground_tilt_deg", 8.0));

    if (useLegacyImuOrientation()) {
      depth_obstacle_filter::ImuFilter::Config imu_cfg;
      imu_filter_ = std::make_unique<depth_obstacle_filter::ImuFilter>(
          imu_cfg, get_logger(), get_clock());
    }

    // ── TF broadcaster ───────────────────────────────────────────────────────
    tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // ── Publicadores principales ─────────────────────────────
    obstacle_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/depth_obstacle_filter/obstacle_cloud", 10);

    camera_height_pub_ = create_publisher<std_msgs::msg::Float32>(
        "/depth_obstacle_filter/camera_height", rclcpp::QoS(1).transient_local());

    if (publish_local_mapper_interface_) {
      obstacle_odom_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
          "/depth_obstacle_filter/obstacle_cloud_odom", 10);

      free_endpoints_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
          "/depth_obstacle_filter/free_endpoints", 10);

      sensor_pos_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
          "/depth_obstacle_filter/sensor_pos", 10);
    }

    if (debug_enabled_) {
      cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
          "/depth_obstacle_filter/debug/depth_cloud", 10);

      raw_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
          "/depth_obstacle_filter/debug/raw_cloud", 10);

      ground_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
          "/depth_obstacle_filter/debug/ground_cloud", 10);

      ceiling_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
          "/depth_obstacle_filter/debug/ceiling_cloud", 10);
    }

    // ── Suscriptores ──────────────────────────────────────────────────────────
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "depth/image", rclcpp::SensorDataQoS(),
        std::bind(&DepthObstacleFilterNode::onDepth, this, _1));

    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "depth/camera_info", rclcpp::SensorDataQoS(),
        std::bind(&DepthObstacleFilterNode::onCameraInfo, this, _1));

    if (useNavOdomOrientation()) {
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
          "nav_odom", rclcpp::QoS(1).reliable(),
          std::bind(&DepthObstacleFilterNode::onOdom, this, _1));
    } else {
      imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
          "imu", rclcpp::SensorDataQoS(),
          std::bind(&DepthObstacleFilterNode::onImu, this, _1));
    }

    RCLCPP_INFO(get_logger(),
        "DepthObstacleFilterNode listo. range=[%.2f, %.2f]m debug=%s local_mapper_interface=%s orientation_source=%s",
        range_min_m_, range_max_m_,
        debug_enabled_ ? "true" : "false",
        publish_local_mapper_interface_ ? "true" : "false",
        orientation_source_.c_str());
    last_perf_log_ = get_clock()->now();
  }

 private:
  bool useNavOdomOrientation() const {
    return orientation_source_ == "nav_odom";
  }

  bool useLegacyImuOrientation() const {
    return orientation_source_ == "imu_legacy";
  }

  static double elapsedMs(const struct timespec& start) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (now.tv_sec - start.tv_sec) * 1000.0 +
           (now.tv_nsec - start.tv_nsec) / 1e6;
  }

  void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (!projector_) {
      projector_ = std::make_unique<depth_obstacle_filter::DepthProjector>(*msg);
      RCLCPP_INFO(get_logger(),
          "CameraInfo recibida: fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
          projector_->fx(), projector_->fy(),
          projector_->cx(), projector_->cy());
    }
  }

  bool updateCameraHeight(float raw_height_m) {
    if (!std::isfinite(raw_height_m) ||
        raw_height_m <= height_min_valid_m_ ||
        raw_height_m >= height_max_valid_m_) {
      return false;
    }

    if (!height_filter_initialized_) {
      filtered_camera_height_m_ = raw_height_m;
      floor_height_m_ = filtered_camera_height_m_;
      height_filter_initialized_ = true;
      return true;
    }

    const float delta = raw_height_m - filtered_camera_height_m_;
    if (std::abs(delta) > height_outlier_reject_m_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "[camera_height] medicion rechazada raw=%.3f filtrada=%.3f delta=%.3f",
          raw_height_m, filtered_camera_height_m_, delta);
      return false;
    }

    const float limited_step = std::clamp(
        height_filter_alpha_ * delta,
        -height_max_step_m_,
        height_max_step_m_);
    if (std::abs(limited_step) < 1e-4f) {
      return false;
    }

    filtered_camera_height_m_ += limited_step;
    floor_height_m_ = filtered_camera_height_m_;
    return true;
  }

  bool isGroundReliableForHeight() const {
    const auto& plane = ground_estimator_->groundPlane();
    if (!plane.valid) return false;
    if (plane.quality < height_min_ground_quality_) return false;
    if (plane.n_inliers < height_min_ground_inliers_) return false;

    const float cos_tilt = std::abs(plane.ny);
    const float tilt_deg = std::acos(std::clamp(cos_tilt, 0.0f, 1.0f))
                           * 180.0f / static_cast<float>(M_PI);
    return tilt_deg <= height_max_ground_tilt_deg_;
  }

  void logRejectedHeightByGroundConfidence() {
    const auto& plane = ground_estimator_->groundPlane();
    const float cos_tilt = std::abs(plane.ny);
    const float tilt_deg = plane.valid
        ? std::acos(std::clamp(cos_tilt, 0.0f, 1.0f)) *
              180.0f / static_cast<float>(M_PI)
        : 90.0f;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "[camera_height] conservada: piso poco confiable valid=%d quality=%.3f inliers=%d tilt=%.1fdeg",
        plane.valid ? 1 : 0,
        plane.quality,
        plane.n_inliers,
        tilt_deg);
  }

  void maybePublishCameraHeight() {
    if (!height_filter_initialized_) return;

    const bool first_publish = !height_publish_initialized_;
    const float delta = std::abs(
        filtered_camera_height_m_ - last_published_height_);
    if (!first_publish && delta < height_publish_delta_m_) {
      return;
    }

    std_msgs::msg::Float32 height_msg;
    height_msg.data = filtered_camera_height_m_;
    camera_height_pub_->publish(height_msg);
    RCLCPP_INFO(get_logger(),
        "[camera_height] raw/filtrada %.3f/%.3f m publicada (delta=%.3f m)",
        ground_estimator_->cameraHeightM(),
        filtered_camera_height_m_,
        first_publish ? 0.0f : delta);
    last_published_height_ = filtered_camera_height_m_;
    height_publish_initialized_ = true;
  }

  void publishOrientationTransforms(const rclcpp::Time& stamp) {
    // ── TF 1: odom → gravity_aligned_frame ───────────────────────────────────
    geometry_msgs::msg::TransformStamped tf_gaf;
    tf_gaf.header.stamp    = stamp;
    tf_gaf.header.frame_id = "odom";
    tf_gaf.child_frame_id  = "gravity_aligned_frame";
    tf_gaf.transform.translation.x = odom_pos_x_;
    tf_gaf.transform.translation.y = odom_pos_y_;
    tf_gaf.transform.translation.z = odom_pos_z_;
    tf_gaf.transform.rotation.w = q_yaw_yd_cached_.w;
    tf_gaf.transform.rotation.x = q_yaw_yd_cached_.x;
    tf_gaf.transform.rotation.y = q_yaw_yd_cached_.y;
    tf_gaf.transform.rotation.z = q_yaw_yd_cached_.z;
    tf_br_->sendTransform(tf_gaf);

    // ── TF 2: gravity_aligned_frame → camera_depth_optical_frame ─────────────
    geometry_msgs::msg::TransformStamped tf_cam;
    tf_cam.header.stamp    = stamp;
    tf_cam.header.frame_id = "gravity_aligned_frame";
    tf_cam.child_frame_id  = "camera_depth_optical_frame";
    tf_cam.transform.rotation.w = q_rp_.w;
    tf_cam.transform.rotation.x = q_rp_.x;
    tf_cam.transform.rotation.y = q_rp_.y;
    tf_cam.transform.rotation.z = q_rp_.z;
    tf_br_->sendTransform(tf_cam);
  }

  // ── LEGACY: GravityAligner (E4) ──────────────────────────────────────────
  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg) {
    if (!imu_filter_) return;

    const auto& a = msg->linear_acceleration;
    imu_filter_->processAccel(
        static_cast<float>(a.x),
        static_cast<float>(a.y),
        static_cast<float>(a.z));

    if (imu_filter_->isDynamic()) {
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 500,
          "IMU dinamica: conservando ultima orientacion gravity_aligned");
      return;
    }

    q_rp_ = aligner_.estimateOrientation(
        imu_filter_->ax(),
        imu_filter_->ay(),
        imu_filter_->az()).normalized();
    if (q_rp_.w < 0.f) {
      q_rp_ = {-q_rp_.w, -q_rp_.x, -q_rp_.y, -q_rp_.z};
    }
    q_ = q_rp_;
    q_yaw_yd_cached_ = {1.f, 0.f, 0.f, 0.f};
    odom_pos_x_ = 0.0f;
    odom_pos_y_ = 0.0f;
    odom_pos_z_ = 0.0f;
  }

  // ── Orientación VIO (fuente principal) ───────────────────────────────────
  // Calcula q_rp_ (roll+pitch) y q_yaw_yd_cached_ a partir de nav_odom.
  // Publica dos TFs:
  //   1) odom → gravity_aligned_frame  (posición + solo yaw)
  //   2) gravity_aligned_frame → camera_depth_optical_frame  (roll+pitch)
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    q_ = nav_math::Quaternion{
        static_cast<float>(msg->pose.pose.orientation.w),
        static_cast<float>(msg->pose.pose.orientation.x),
        static_cast<float>(msg->pose.pose.orientation.y),
        static_cast<float>(msg->pose.pose.orientation.z)}.normalized();

    // ── Yaw en Y-down ─────────────────────────────────────────────────────────
    const nav_math::Vec3 fwd = q_.rotate({0.f, 0.f, 1.f});
    const float fwd_xz = std::sqrt(fwd.x * fwd.x + fwd.z * fwd.z);
    nav_math::Quaternion q_yaw_yd{1.f, 0.f, 0.f, 0.f};
    if (fwd_xz > 1e-4f) {
      const float yaw = std::atan2(fwd.x / fwd_xz, fwd.z / fwd_xz);
      const float hy  = yaw * 0.5f;
      q_yaw_yd = {std::cos(hy), 0.f, std::sin(hy), 0.f};
    }

    // ── Roll+pitch sin yaw ────────────────────────────────────────────────────
    q_rp_ = (q_yaw_yd.conjugate() * q_).normalized();
    if (q_rp_.w < 0.f) {
      q_rp_ = {-q_rp_.w, -q_rp_.x, -q_rp_.y, -q_rp_.z};
    }

    odom_pos_x_ = static_cast<float>(msg->pose.pose.position.x);
    odom_pos_y_ = static_cast<float>(msg->pose.pose.position.y);
    odom_pos_z_ = static_cast<float>(msg->pose.pose.position.z);
    q_yaw_yd_cached_ = q_yaw_yd;

    publishOrientationTransforms(msg->header.stamp);
  }

  void onDepth(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!projector_) return;

    if (msg->encoding != "16UC1") {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Encoding inesperado: %s (se espera 16UC1)", msg->encoding.c_str());
      return;
    }

    if (useLegacyImuOrientation()) {
      publishOrientationTransforms(msg->header.stamp);
    }

    // Salida anticipada si nadie escucha (ahorra CPU en pruebas sin suscriptores).
    const bool has_obstacle_subs =
        obstacle_pub_->get_subscription_count() > 0;
    const bool has_height_subs =
        camera_height_pub_->get_subscription_count() > 0;
    const bool has_local_mapper_subs = publish_local_mapper_interface_ &&
        ((obstacle_odom_pub_ &&
          obstacle_odom_pub_->get_subscription_count() > 0) ||
         (free_endpoints_pub_ &&
          free_endpoints_pub_->get_subscription_count() > 0) ||
         (sensor_pos_pub_ &&
          sensor_pos_pub_->get_subscription_count() > 0));
    const bool has_debug_subs = debug_enabled_ &&
        ((cloud_pub_ && cloud_pub_->get_subscription_count() > 0) ||
         (raw_cloud_pub_ && raw_cloud_pub_->get_subscription_count() > 0) ||
         (ground_pub_ && ground_pub_->get_subscription_count() > 0) ||
         (ceiling_pub_ && ceiling_pub_->get_subscription_count() > 0));
    if (!has_obstacle_subs && !has_height_subs &&
        !has_local_mapper_subs && !has_debug_subs) return;

    struct timespec t_total;
    struct timespec t_stage;
    clock_gettime(CLOCK_MONOTONIC, &t_total);

    // ── Retroproyección ───────────────────────────────────────────────────────
    clock_gettime(CLOCK_MONOTONIC, &t_stage);
    const auto* depth_data = reinterpret_cast<const uint16_t*>(msg->data.data());
    auto points = projector_->projectDepthImage(
        depth_data, msg->width, msg->height, 1e-3f, depth_pixel_stride_);
    const double t_project_ms = elapsedMs(t_stage);

    // ── Filtrado de rango ─────────────────────────────────────────────────────
    // Puntos en [range_min_m_, range_max_m_) → 'valid' (obstáculos y suelo posibles).
    // Puntos >= range_max_m_ → 'beyond_range_pts' submuestreados 1/16:
    //   cubrían el FOV con ~1200 rayos para marcar espacio libre hasta el borde.
    std::vector<depth_obstacle_filter::DepthProjector::Point3D> valid;
    valid.reserve(points.size() / 4);
    std::vector<depth_obstacle_filter::DepthProjector::Point3D> beyond_range_pts;
    int beyond_stride_counter = 0;
    int nan_count = 0;
    int below_min_count = 0;
    int beyond_count = 0;
    clock_gettime(CLOCK_MONOTONIC, &t_stage);
    for (const auto& pt : points) {
      if (std::isnan(pt.z)) {
        ++nan_count;
        continue;
      }
      if (pt.z <= range_min_m_) {
        ++below_min_count;
        continue;
      }
      if (pt.z >= range_max_m_) {
        ++beyond_count;
        if (publish_local_mapper_interface_ &&
            (beyond_stride_counter++ & 15) == 0) {
          beyond_range_pts.push_back(pt);
        }
        continue;
      }
      valid.push_back(pt);
    }
    const double t_range_ms = elapsedMs(t_stage);

    // ── Debug: nube cruda en camera_depth_optical_frame ──────────────────────
    if (cloud_pub_ && cloud_pub_->get_subscription_count() > 0) {
      publishCloud(cloud_pub_, msg->header, valid);
    }

    // ── Debug: nube cruda en gravity_aligned_frame (oscila al inclinar) ───────
    if (raw_cloud_pub_ && raw_cloud_pub_->get_subscription_count() > 0) {
      std_msgs::msg::Header raw_hdr;
      raw_hdr.stamp    = msg->header.stamp;
      raw_hdr.frame_id = "gravity_aligned_frame";
      publishCloud(raw_cloud_pub_, raw_hdr, valid);
    }

    // ── Estimación del suelo y publicación de nubes de salida ─────────────────
    {
      clock_gettime(CLOCK_MONOTONIC, &t_stage);
      // Rotar a gravity_aligned_frame con la orientación VIO cacheada.
      std::vector<depth_obstacle_filter::GroundEstimator::Point3D> ge_cloud;
      ge_cloud.reserve(valid.size());
      for (const auto& pt : valid) {
        const auto r = q_rp_.rotate(nav_math::Vec3{pt.x, pt.y, pt.z});
        ge_cloud.push_back({r.x, r.y, r.z});
      }

      const bool ground_ok = ground_estimator_->estimate(ge_cloud);
      const double t_ground_ms = elapsedMs(t_stage);

      std_msgs::msg::Header aligned_hdr;
      aligned_hdr.stamp    = msg->header.stamp;
      aligned_hdr.frame_id = "gravity_aligned_frame";

      if (ground_pub_ && ground_pub_->get_subscription_count() > 0) {
        publishIndexedCloud(ground_pub_, aligned_hdr, ge_cloud,
                            ground_estimator_->groundIndices());
      }

      if (ceiling_pub_ && ceiling_pub_->get_subscription_count() > 0) {
        publishIndexedCloud(ceiling_pub_, aligned_hdr, ge_cloud,
                            ground_estimator_->ceilingIndices());
      }

      // ── Actualizar altura del suelo ───────────────────────────────────────
      if (ground_ok) {
        if (isGroundReliableForHeight()) {
          const float h = ground_estimator_->cameraHeightM();
          if (updateCameraHeight(h)) {
            maybePublishCameraHeight();
          }
        } else {
          logRejectedHeightByGroundConfidence();
        }
      }

      if (has_obstacle_subs) {
        publishIndexedCloud(obstacle_pub_, aligned_hdr, ge_cloud,
                            ground_estimator_->obstacleIndices());
      }

      // ── Construir y publicar nubes en frame odom ──────────────────────────
      // Los tres topics comparten el mismo stamp → ExactTimeSynchronizer en
      // local_mapper garantiza que cada actualización del mapa usa exactamente
      // los datos del mismo frame de profundidad.
      {
        clock_gettime(CLOCK_MONOTONIC, &t_stage);
        std_msgs::msg::Header odom_hdr;
        odom_hdr.stamp    = msg->header.stamp;
        odom_hdr.frame_id = "odom";

        if (publish_local_mapper_interface_) {
          // Obstáculos → frame odom (preserva Y=altura para visualización 3D)
          const auto& obs_idx = ground_estimator_->obstacleIndices();
          std::vector<depth_obstacle_filter::DepthProjector::Point3D> obs_odom;
          obs_odom.reserve(obs_idx.size());
          for (int idx : obs_idx) {
            const auto& p = ge_cloud[idx];
            const nav_math::Vec3 p_odom =
                q_yaw_yd_cached_.rotate({p.x, 0.f, p.z});
            obs_odom.push_back({p_odom.x + odom_pos_x_, p.y,
                                p_odom.z + odom_pos_z_});
          }
          publishCloud(obstacle_odom_pub_, odom_hdr, obs_odom);
        }

        if (publish_local_mapper_interface_) {
          // Endpoints de rayos libres → frame odom (suelo + techo submuestreado
          // 1/4, más puntos más allá del rango submuestreados 1/16).
          // En odom frame: Y=0 (solo XZ importa para el mapa 2D).
          std::vector<depth_obstacle_filter::DepthProjector::Point3D> free_odom;

          const auto addFreeFromIndices =
              [&](const std::vector<int>& indices, int stride) {
                for (int k = 0; k < static_cast<int>(indices.size());
                     k += stride) {
                  const auto& p = ge_cloud[indices[k]];
                  const nav_math::Vec3 p_odom =
                      q_yaw_yd_cached_.rotate({p.x, 0.f, p.z});
                  free_odom.push_back({p_odom.x + odom_pos_x_, 0.f,
                                       p_odom.z + odom_pos_z_});
                }
              };
          addFreeFromIndices(ground_estimator_->groundIndices(),   4);
          addFreeFromIndices(ground_estimator_->ceilingIndices(),  4);

          for (const auto& pt : beyond_range_pts) {
            const float scale = range_max_m_ / pt.z;
            const nav_math::Vec3 p_gaf =
                q_rp_.rotate({pt.x * scale, pt.y * scale, pt.z * scale});
            const nav_math::Vec3 p_odom =
                q_yaw_yd_cached_.rotate({p_gaf.x, 0.f, p_gaf.z});
            free_odom.push_back(
                {p_odom.x + odom_pos_x_, 0.f, p_odom.z + odom_pos_z_});
          }
          publishCloud(free_endpoints_pub_, odom_hdr, free_odom);

          // Posición del sensor en odom.
          // point.x/z = posición XZ en odom.
          // point.y   = altura del suelo sobre la cámara (m); 0 si no detectado.
          //             local_mapper la usa para colocar el OccupancyGrid a la
          //             altura correcta del suelo en RViz.
          geometry_msgs::msg::PointStamped sensor_pos_msg;
          sensor_pos_msg.header = odom_hdr;
          sensor_pos_msg.point.x = static_cast<double>(odom_pos_x_);
          sensor_pos_msg.point.y = static_cast<double>(floor_height_m_);
          sensor_pos_msg.point.z = static_cast<double>(odom_pos_z_);
          sensor_pos_pub_->publish(sensor_pos_msg);
        }

        const double t_publish_ms = elapsedMs(t_stage);
        const double t_total_ms = elapsedMs(t_total);
        const auto now = get_clock()->now();
        const auto msg_stamp = rclcpp::Time(msg->header.stamp);
        const double msg_age_ms = (now - msg_stamp).nanoseconds() / 1e6;
        const size_t obs_count = ground_estimator_->obstacleIndices().size();
        const size_t ground_count = ground_estimator_->groundIndices().size();
        const size_t ceiling_count = ground_estimator_->ceilingIndices().size();

        perf_ext_.frames++;
        perf_ext_.sum_project_ms += t_project_ms;
        perf_ext_.max_project_ms = std::max(perf_ext_.max_project_ms, t_project_ms);
        perf_ext_.sum_range_ms += t_range_ms;
        perf_ext_.max_range_ms = std::max(perf_ext_.max_range_ms, t_range_ms);
        perf_ext_.sum_ground_ms += t_ground_ms;
        perf_ext_.max_ground_ms = std::max(perf_ext_.max_ground_ms, t_ground_ms);
        perf_ext_.sum_publish_ms += t_publish_ms;
        perf_ext_.max_publish_ms = std::max(perf_ext_.max_publish_ms, t_publish_ms);
        perf_ext_.sum_total_ms += t_total_ms;
        perf_ext_.max_total_ms = std::max(perf_ext_.max_total_ms, t_total_ms);
        perf_ext_.sum_age_ms += msg_age_ms;
        perf_ext_.sum_points_projected += points.size();
        perf_ext_.sum_valid += valid.size();
        perf_ext_.sum_beyond += beyond_range_pts.size();
        perf_ext_.sum_nan += nan_count;
        perf_ext_.sum_below_min += below_min_count;
        perf_ext_.sum_beyond_raw += beyond_count;
        perf_ext_.sum_obstacles += obs_count;
        perf_ext_.sum_ground += ground_count;
        perf_ext_.sum_ceiling += ceiling_count;

        if (obs_count == 0) {
          ++empty_obstacle_streak_;
          ++perf_ext_.empty_obstacle_frames;
          if (empty_obstacle_streak_ == 1 ||
              empty_obstacle_streak_ % empty_obstacle_warn_every_ == 0) {
            RCLCPP_WARN(get_logger(),
                "[empty_obstacles] streak=%d age=%.1fms projected=%zu valid=%zu nan=%d below_min=%d beyond_raw=%d beyond_kept=%zu ground_ok=%d ground=%zu ceiling=%zu",
                empty_obstacle_streak_,
                msg_age_ms,
                points.size(),
                valid.size(),
                nan_count,
                below_min_count,
                beyond_count,
                beyond_range_pts.size(),
                ground_ok ? 1 : 0,
                ground_count,
                ceiling_count);
          }
        } else {
          empty_obstacle_streak_ = 0;
        }

        if (perf_log_enabled_ &&
            (now - last_perf_log_).seconds() >= perf_log_period_s_ &&
            perf_ext_.frames > 0) {
          const double nf = static_cast<double>(perf_ext_.frames);
          const auto& a = perf_accum_;
          RCLCPP_INFO(get_logger(),
              "[PERF %.1fs] frames=%d empty_obs=%d age=%.1fms | proj=%.2f/%.2fms range=%.2f/%.2fms ground=%.2f/%.2fms publish=%.2f/%.2fms total=%.2f/%.2fms | pts proj=%.0f valid=%.0f beyond=%.0f nan=%.0f below=%.0f beyond_raw=%.0f obs=%.0f ground=%.0f ceil=%.0f | GE total=%.2fms voxel=%.2fms ransac=%.2fms refine=%.2fms",
              perf_log_period_s_,
              perf_ext_.frames,
              perf_ext_.empty_obstacle_frames,
              perf_ext_.sum_age_ms / nf,
              perf_ext_.sum_project_ms / nf, perf_ext_.max_project_ms,
              perf_ext_.sum_range_ms / nf, perf_ext_.max_range_ms,
              perf_ext_.sum_ground_ms / nf, perf_ext_.max_ground_ms,
              perf_ext_.sum_publish_ms / nf, perf_ext_.max_publish_ms,
              perf_ext_.sum_total_ms / nf, perf_ext_.max_total_ms,
              static_cast<double>(perf_ext_.sum_points_projected) / nf,
              static_cast<double>(perf_ext_.sum_valid) / nf,
              static_cast<double>(perf_ext_.sum_beyond) / nf,
              static_cast<double>(perf_ext_.sum_nan) / nf,
              static_cast<double>(perf_ext_.sum_below_min) / nf,
              static_cast<double>(perf_ext_.sum_beyond_raw) / nf,
              static_cast<double>(perf_ext_.sum_obstacles) / nf,
              static_cast<double>(perf_ext_.sum_ground) / nf,
              static_cast<double>(perf_ext_.sum_ceiling) / nf,
              a.frames > 0 ? a.sum_total_ms / a.frames : 0.0,
              a.frames > 0 ? a.sum_voxel_ms / a.frames : 0.0,
              a.frames > 0 ? a.sum_ransac_ms / a.frames : 0.0,
              a.frames > 0 ? a.sum_refine_ms / a.frames : 0.0);
          perf_ext_.reset();
          perf_accum_.reset();
          last_perf_log_ = now;
        }
      }

      // ── Diagnóstico ───────────────────────────────────────────────────────
      perf_accum_.accumulate(ground_estimator_->perfStats());

      if (!ground_ok) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Suelo no detectado. voxel=%d/%d",
            ground_estimator_->perfStats().n_voxel,
            ground_estimator_->perfStats().n_input);
      }

    }
  }

  // ── Helpers ───────────────────────────────────────────────────────────────

  static sensor_msgs::msg::PointCloud2 makeCloudMsg(
      const std_msgs::msg::Header& header, uint32_t n_points)
  {
    sensor_msgs::msg::PointCloud2 pc;
    pc.header = header;
    pc.height = 1;
    pc.width = n_points;
    pc.is_dense = true;
    pc.is_bigendian = false;
    pc.point_step = 12;  // 3 × float32
    pc.row_step = pc.point_step * pc.width;
    auto add = [&](const std::string& name, uint32_t offset) {
      sensor_msgs::msg::PointField f;
      f.name = name; f.offset = offset;
      f.datatype = sensor_msgs::msg::PointField::FLOAT32; f.count = 1;
      pc.fields.push_back(f);
    };
    add("x", 0); add("y", 4); add("z", 8);
    pc.data.resize(pc.row_step);
    return pc;
  }

  void publishCloud(
      const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub,
      const std_msgs::msg::Header& header,
      const std::vector<depth_obstacle_filter::DepthProjector::Point3D>& pts)
  {
    auto pc = makeCloudMsg(header, static_cast<uint32_t>(pts.size()));
    auto* ptr = reinterpret_cast<float*>(pc.data.data());
    for (const auto& pt : pts) { *ptr++ = pt.x; *ptr++ = pt.y; *ptr++ = pt.z; }
    pub->publish(std::move(pc));
  }

  void publishIndexedCloud(
      const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub,
      const std_msgs::msg::Header& header,
      const std::vector<depth_obstacle_filter::GroundEstimator::Point3D>& cloud,
      const std::vector<int>& indices)
  {
    auto pc = makeCloudMsg(header, static_cast<uint32_t>(indices.size()));
    auto* ptr = reinterpret_cast<float*>(pc.data.data());
    for (int i : indices) {
      *ptr++ = cloud[i].x; *ptr++ = cloud[i].y; *ptr++ = cloud[i].z;
    }
    pub->publish(std::move(pc));
  }

  // ── Acumulador de performance ─────────────────────────────────────────────
  struct PerfAccum {
    int    frames     = 0;
    double sum_diag_ms   = 0, max_diag_ms   = 0;
    double sum_voxel_ms  = 0, max_voxel_ms  = 0;
    double sum_ransac_ms = 0, max_ransac_ms = 0;
    double sum_refine_ms = 0, max_refine_ms = 0;
    double sum_total_ms  = 0, max_total_ms  = 0;
    double sum_input  = 0;
    double sum_voxel  = 0;

    void reset() { *this = PerfAccum{}; }

    void accumulate(const depth_obstacle_filter::GroundEstimator::PerfStats& p) {
      ++frames;
      sum_diag_ms   += p.time_diag_ms;
      max_diag_ms    = p.time_diag_ms   > max_diag_ms   ? p.time_diag_ms   : max_diag_ms;
      sum_voxel_ms  += p.time_voxel_ms;
      max_voxel_ms   = p.time_voxel_ms  > max_voxel_ms  ? p.time_voxel_ms  : max_voxel_ms;
      sum_ransac_ms += p.time_ransac_ms;
      max_ransac_ms  = p.time_ransac_ms > max_ransac_ms ? p.time_ransac_ms : max_ransac_ms;
      sum_refine_ms += p.time_refine_ms;
      max_refine_ms  = p.time_refine_ms > max_refine_ms ? p.time_refine_ms : max_refine_ms;
      sum_total_ms  += p.time_total_ms;
      max_total_ms   = p.time_total_ms  > max_total_ms  ? p.time_total_ms  : max_total_ms;
      sum_input     += p.n_input;
      sum_voxel     += p.n_voxel;
    }
  };

  struct PerfExtAccum {
    int frames = 0;
    int empty_obstacle_frames = 0;
    double sum_project_ms = 0, max_project_ms = 0;
    double sum_range_ms = 0, max_range_ms = 0;
    double sum_ground_ms = 0, max_ground_ms = 0;
    double sum_publish_ms = 0, max_publish_ms = 0;
    double sum_total_ms = 0, max_total_ms = 0;
    double sum_age_ms = 0;
    uint64_t sum_points_projected = 0;
    uint64_t sum_valid = 0;
    uint64_t sum_beyond = 0;
    uint64_t sum_nan = 0;
    uint64_t sum_below_min = 0;
    uint64_t sum_beyond_raw = 0;
    uint64_t sum_obstacles = 0;
    uint64_t sum_ground = 0;
    uint64_t sum_ceiling = 0;

    void reset() { *this = PerfExtAccum{}; }
  };

  // ── Miembros ──────────────────────────────────────────────────────────────
  std::unique_ptr<depth_obstacle_filter::DepthProjector>   projector_;
  std::unique_ptr<depth_obstacle_filter::ImuFilter>        imu_filter_;
  std::unique_ptr<depth_obstacle_filter::GroundEstimator>  ground_estimator_;
  depth_obstacle_filter::GravityAligner                    aligner_;
  nav_math::Quaternion                            q_{1.0f, 0.0f, 0.0f, 0.0f};
  nav_math::Quaternion                            q_rp_{1.0f, 0.0f, 0.0f, 0.0f};
  nav_math::Quaternion                            q_yaw_yd_cached_{1.0f, 0.0f, 0.0f, 0.0f};
  float                                           odom_pos_x_{0.0f};
  float                                           odom_pos_y_{0.0f};
  float                                           odom_pos_z_{0.0f};
  float                                           floor_height_m_{0.0f};
  float                                           filtered_camera_height_m_{0.0f};
  float                                           last_published_height_{0.0f};
  bool                                            height_filter_initialized_{false};
  bool                                            height_publish_initialized_{false};
  std::shared_ptr<tf2_ros::TransformBroadcaster>  tf_br_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr        imu_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    obstacle_pub_;

  // Interface hacia local_mapper (tres topics sincronizados por stamp)
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    obstacle_odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    free_endpoints_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr sensor_pos_pub_;

  // Debug / visualización
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ceiling_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr         camera_height_pub_;

  float range_min_m_ = 0.1f;
  float range_max_m_ = 5.0f;
  int depth_pixel_stride_ = 1;
  bool perf_log_enabled_ = false;
  bool debug_enabled_ = false;
  bool publish_local_mapper_interface_ = false;
  std::string orientation_source_ = "nav_odom";
  double perf_log_period_s_ = 5.0;
  int empty_obstacle_warn_every_ = 5;
  int empty_obstacle_streak_ = 0;
  float height_filter_alpha_ = 0.03f;
  float height_max_step_m_ = 0.005f;
  float height_outlier_reject_m_ = 0.25f;
  float height_publish_delta_m_ = 0.05f;
  float height_min_ground_quality_ = 0.35f;
  int height_min_ground_inliers_ = 50;
  float height_max_ground_tilt_deg_ = 8.0f;
  float height_min_valid_m_ = 0.5f;
  float height_max_valid_m_ = 2.5f;
  PerfAccum perf_accum_;
  PerfExtAccum perf_ext_;
  rclcpp::Time last_perf_log_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthObstacleFilterNode>());
  rclcpp::shutdown();
  return 0;
}
