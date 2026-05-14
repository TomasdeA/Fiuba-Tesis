// ─────────────────────────────────────────────────────────────────────────────
// DepthObstacleFilterNode
//
// Suscribe:  depth/image  (sensor_msgs/Image, 16UC1)
//            depth/camera_info (sensor_msgs/CameraInfo)
//            nav_odom     (nav_msgs/Odometry) — orientación VIO de nav_odometry
//            imu          [LEGACY — conservado para Experimento E4]
//
// Publica (interface con local_mapper — tres topics sincronizados por stamp):
//   /depth_obstacle_filter/obstacle_cloud  — obstáculos en frame odom (PointCloud2 XYZ)
//   /depth_obstacle_filter/free_endpoints  — endpoints de rayos libres en frame odom
//   /depth_obstacle_filter/sensor_pos      — posición de la cámara en odom (PointStamped)
//                                            point.y = altura estimada del suelo (m)
//
// Publica (debug / visualización):
//   /depth_obstacle_filter/debug/depth_cloud   — nube cruda en camera_depth_optical_frame
//   /depth_obstacle_filter/debug/raw_cloud     — nube cruda en gravity_aligned_frame
//   /depth_obstacle_filter/debug/ground_cloud  — suelo en gravity_aligned_frame
//   /depth_obstacle_filter/debug/ceiling_cloud — techo descartado en gravity_aligned_frame
//   /depth_obstacle_filter/camera_height       — altura de cámara sobre el suelo (Float32)
//
// Publica TFs:
//   odom → gravity_aligned_frame          (yaw + traslación desde nav_odom)
//   gravity_aligned_frame → camera_depth_optical_frame  (roll + pitch)
//
// Los tres topics de interface (obstacle_cloud, free_endpoints, sensor_pos)
// comparten siempre el mismo header.stamp (= stamp del frame de profundidad).
// local_mapper usa ExactTimeSynchronizer sobre los tres para garantizar
// sincronización perfecta: cada actualización del mapa usa exactamente
// los obstáculos y la posición del sensor del mismo instante.
// ─────────────────────────────────────────────────────────────────────────────

#include <rclcpp/rclcpp.hpp>
#include <limits>
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

    depth_obstacle_filter::GroundEstimator::Config ge_cfg;
    ge_cfg.voxel_size_m   = static_cast<float>(
        declare_parameter<double>("voxel_size_m", 0.05));
    ge_cfg.ransac_max_iter = declare_parameter<int>("ransac_max_iter", 100);
    ge_cfg.ransac_inlier_tol = static_cast<float>(
        declare_parameter<double>("ground_inlier_tol", 0.0));
    ge_cfg.min_person_height_m = static_cast<float>(
        declare_parameter<double>("min_person_height_m", 1.3));
    ge_cfg.ceiling_delta_m = static_cast<float>(
        declare_parameter<double>("ceiling_delta_m", 0.1));
    ground_estimator_ = std::make_unique<depth_obstacle_filter::GroundEstimator>(ge_cfg);

    // ── ImuFilter [LEGACY] ────────────────────────────────────────────────────
    depth_obstacle_filter::ImuFilter::Config imu_cfg;
    imu_filter_ = std::make_unique<depth_obstacle_filter::ImuFilter>(
        imu_cfg, get_logger(), get_clock());

    // ── TF broadcaster ────────────────────────────────────────────────────────
    tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // ── Publicadores — interface con local_mapper ─────────────────────────────
    obstacle_odom_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/depth_obstacle_filter/obstacle_cloud", 10);

    free_endpoints_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/depth_obstacle_filter/free_endpoints", 10);

    sensor_pos_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
        "/depth_obstacle_filter/sensor_pos", 10);

    // ── Publicadores — debug / visualización ─────────────────────────────────
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/depth_obstacle_filter/debug/depth_cloud", 10);

    raw_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/depth_obstacle_filter/debug/raw_cloud", 10);

    ground_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/depth_obstacle_filter/debug/ground_cloud", 10);

    ceiling_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/depth_obstacle_filter/debug/ceiling_cloud", 10);

    // Altura de la cámara sobre el suelo (calibración, QoS transient_local).
    camera_height_pub_ = create_publisher<std_msgs::msg::Float32>(
        "/depth_obstacle_filter/camera_height", rclcpp::QoS(1).transient_local());

    // ── Suscriptores ──────────────────────────────────────────────────────────
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "depth/image", rclcpp::SensorDataQoS(),
        std::bind(&DepthObstacleFilterNode::onDepth, this, _1));

    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "depth/camera_info", rclcpp::SensorDataQoS(),
        std::bind(&DepthObstacleFilterNode::onCameraInfo, this, _1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "nav_odom", rclcpp::QoS(1).reliable(),
        std::bind(&DepthObstacleFilterNode::onOdom, this, _1));

    // [LEGACY] — conservado para Experimento E4
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu", rclcpp::SensorDataQoS(),
        std::bind(&DepthObstacleFilterNode::onImu, this, _1));

    RCLCPP_INFO(get_logger(),
        "DepthObstacleFilterNode listo. range=[%.2f, %.2f]m",
        range_min_m_, range_max_m_);
    last_perf_log_ = get_clock()->now();
  }

 private:
  void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (!projector_) {
      projector_ = std::make_unique<depth_obstacle_filter::DepthProjector>(*msg);
      RCLCPP_INFO(get_logger(),
          "CameraInfo recibida: fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
          projector_->fx(), projector_->fy(),
          projector_->cx(), projector_->cy());
    }
  }

  // ── LEGACY: GravityAligner ────────────────────────────────────────────────
  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg) {
    (void)msg;
    // ── LEGACY (descomentar para Experimento E4) ──────────────────────────
    // const auto& a = msg->linear_acceleration;
    // imu_filter_->processAccel(...);
    // q_ = aligner_.estimateOrientation(...);
    // ... publicar TF ...
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
    odom_pos_z_ = static_cast<float>(msg->pose.pose.position.z);
    q_yaw_yd_cached_ = q_yaw_yd;

    // ── TF 1: odom → gravity_aligned_frame ───────────────────────────────────
    geometry_msgs::msg::TransformStamped tf_gaf;
    tf_gaf.header.stamp    = msg->header.stamp;
    tf_gaf.header.frame_id = "odom";
    tf_gaf.child_frame_id  = "gravity_aligned_frame";
    tf_gaf.transform.translation.x = msg->pose.pose.position.x;
    tf_gaf.transform.translation.y = msg->pose.pose.position.y;
    tf_gaf.transform.translation.z = msg->pose.pose.position.z;
    tf_gaf.transform.rotation.w = q_yaw_yd.w;
    tf_gaf.transform.rotation.x = q_yaw_yd.x;
    tf_gaf.transform.rotation.y = q_yaw_yd.y;
    tf_gaf.transform.rotation.z = q_yaw_yd.z;
    tf_br_->sendTransform(tf_gaf);

    // ── TF 2: gravity_aligned_frame → camera_depth_optical_frame ─────────────
    geometry_msgs::msg::TransformStamped tf_cam;
    tf_cam.header.stamp    = msg->header.stamp;
    tf_cam.header.frame_id = "gravity_aligned_frame";
    tf_cam.child_frame_id  = "camera_depth_optical_frame";
    tf_cam.transform.rotation.w = q_rp_.w;
    tf_cam.transform.rotation.x = q_rp_.x;
    tf_cam.transform.rotation.y = q_rp_.y;
    tf_cam.transform.rotation.z = q_rp_.z;
    tf_br_->sendTransform(tf_cam);
  }

  void onDepth(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!projector_) return;

    if (msg->encoding != "16UC1") {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Encoding inesperado: %s (se espera 16UC1)", msg->encoding.c_str());
      return;
    }

    // Salida anticipada si nadie escucha (ahorra CPU en pruebas sin suscriptores)
    if (cloud_pub_->get_subscription_count() == 0 &&
        raw_cloud_pub_->get_subscription_count() == 0 &&
        ground_pub_->get_subscription_count() == 0 &&
        obstacle_odom_pub_->get_subscription_count() == 0 &&
        free_endpoints_pub_->get_subscription_count() == 0 &&
        sensor_pos_pub_->get_subscription_count() == 0 &&
        ceiling_pub_->get_subscription_count() == 0) return;

    // ── Retroproyección ───────────────────────────────────────────────────────
    const auto* depth_data = reinterpret_cast<const uint16_t*>(msg->data.data());
    auto points = projector_->projectDepthImage(
        depth_data, msg->width, msg->height);

    // ── Filtrado de rango ─────────────────────────────────────────────────────
    // Puntos en [range_min_m_, range_max_m_) → 'valid' (obstáculos y suelo posibles).
    // Puntos >= range_max_m_ → 'beyond_range_pts' submuestreados 1/16:
    //   cubrían el FOV con ~1200 rayos para marcar espacio libre hasta el borde.
    std::vector<depth_obstacle_filter::DepthProjector::Point3D> valid;
    valid.reserve(points.size() / 4);
    std::vector<depth_obstacle_filter::DepthProjector::Point3D> beyond_range_pts;
    int beyond_stride_counter = 0;
    for (const auto& pt : points) {
      if (std::isnan(pt.z)) continue;
      if (pt.z <= range_min_m_) continue;
      if (pt.z >= range_max_m_) {
        if ((beyond_stride_counter++ & 15) == 0) {
          beyond_range_pts.push_back(pt);
        }
        continue;
      }
      valid.push_back(pt);
    }

    // ── Debug: nube cruda en camera_depth_optical_frame ──────────────────────
    if (cloud_pub_->get_subscription_count() > 0) {
      publishCloud(cloud_pub_, msg->header, valid);
    }

    // ── Debug: nube cruda en gravity_aligned_frame (oscila al inclinar) ───────
    if (raw_cloud_pub_->get_subscription_count() > 0) {
      std_msgs::msg::Header raw_hdr;
      raw_hdr.stamp    = msg->header.stamp;
      raw_hdr.frame_id = "gravity_aligned_frame";
      publishCloud(raw_cloud_pub_, raw_hdr, valid);
    }

    // ── Estimación del suelo y publicación de nubes de salida ─────────────────
    {
      // Rotar a gravity_aligned_frame con la orientación VIO cacheada.
      std::vector<depth_obstacle_filter::GroundEstimator::Point3D> ge_cloud;
      ge_cloud.reserve(valid.size());
      for (const auto& pt : valid) {
        const auto r = q_rp_.rotate(nav_math::Vec3{pt.x, pt.y, pt.z});
        ge_cloud.push_back({r.x, r.y, r.z});
      }

      const bool ground_ok = ground_estimator_->estimate(ge_cloud);

      std_msgs::msg::Header aligned_hdr;
      aligned_hdr.stamp    = msg->header.stamp;
      aligned_hdr.frame_id = "gravity_aligned_frame";

      if (ground_pub_->get_subscription_count() > 0) {
        publishIndexedCloud(ground_pub_, aligned_hdr, ge_cloud,
                            ground_estimator_->groundIndices());
      }

      if (ceiling_pub_->get_subscription_count() > 0) {
        publishIndexedCloud(ceiling_pub_, aligned_hdr, ge_cloud,
                            ground_estimator_->ceilingIndices());
      }

      // ── Actualizar altura del suelo ───────────────────────────────────────
      if (ground_ok) {
        const float h = ground_estimator_->cameraHeightM();
        if (h > 0.5f && h < 2.5f) {
          floor_height_m_ = h;
          if (camera_height_pub_->get_subscription_count() > 0) {
            const float delta = std::abs(h - last_published_height_);
            if (delta > 0.10f) {
              std_msgs::msg::Float32 height_msg;
              height_msg.data = h;
              camera_height_pub_->publish(height_msg);
              RCLCPP_INFO(get_logger(),
                  "[camera_height] %.3f m publicado (delta=%.3f m)",
                  h, delta);
              last_published_height_ = h;
            }
          }
        }
      }

      // ── Construir y publicar nubes en frame odom ──────────────────────────
      // Los tres topics comparten el mismo stamp → ExactTimeSynchronizer en
      // local_mapper garantiza que cada actualización del mapa usa exactamente
      // los datos del mismo frame de profundidad.
      {
        std_msgs::msg::Header odom_hdr;
        odom_hdr.stamp    = msg->header.stamp;
        odom_hdr.frame_id = "odom";

        // Obstáculos → frame odom (preserva Y=altura para visualización 3D)
        {
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

        // Endpoints de rayos libres → frame odom (suelo + techo submuestreado
        // 1/4, más puntos más allá del rango submuestreados 1/16).
        // En odom frame: Y=0 (solo XZ importa para el mapa 2D).
        {
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
        }

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

      // ── Diagnóstico ───────────────────────────────────────────────────────
      perf_accum_.accumulate(ground_estimator_->perfStats());

      if (!ground_ok) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Suelo no detectado. voxel=%d/%d",
            ground_estimator_->perfStats().n_voxel,
            ground_estimator_->perfStats().n_input);
      }

      const auto now = get_clock()->now();
      if (((now - last_perf_log_).seconds() >= 10.0) && false) {
        const auto& a = perf_accum_;
        if (a.frames > 0) {
          const double nf = static_cast<double>(a.frames);
          RCLCPP_INFO(get_logger(),
              "[PERF 10s] frames=%d\n"
              "  Stage1 diagnóstico: avg=%.2fms  max=%.2fms\n"
              "  Stage2 voxel:       avg=%.2fms  max=%.2fms\n"
              "  Stage4 RANSAC:      avg=%.2fms  max=%.2fms\n"
              "  Stage6 refine:      avg=%.2fms  max=%.2fms\n"
              "  Total pipeline:     avg=%.2fms  max=%.2fms\n"
              "  Puntos: avg_entrada=%.0f  avg_voxel=%.0f  ratio=%.1f%%",
              a.frames,
              a.sum_diag_ms/nf,   a.max_diag_ms,
              a.sum_voxel_ms/nf,  a.max_voxel_ms,
              a.sum_ransac_ms/nf, a.max_ransac_ms,
              a.sum_refine_ms/nf, a.max_refine_ms,
              a.sum_total_ms/nf,  a.max_total_ms,
              a.sum_input/nf,     a.sum_voxel/nf,
              100.0 * a.sum_voxel / (a.sum_input > 0.0 ? a.sum_input : 1.0));
          perf_accum_.reset();
          last_perf_log_ = now;
        }
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

  // ── Miembros ──────────────────────────────────────────────────────────────
  std::unique_ptr<depth_obstacle_filter::DepthProjector>   projector_;
  std::unique_ptr<depth_obstacle_filter::ImuFilter>        imu_filter_;   // LEGACY: E4
  std::unique_ptr<depth_obstacle_filter::GroundEstimator>  ground_estimator_;
  depth_obstacle_filter::GravityAligner                    aligner_;      // LEGACY: E4
  nav_math::Quaternion                            q_{1.0f, 0.0f, 0.0f, 0.0f};
  nav_math::Quaternion                            q_rp_{1.0f, 0.0f, 0.0f, 0.0f};
  nav_math::Quaternion                            q_yaw_yd_cached_{1.0f, 0.0f, 0.0f, 0.0f};
  float                                           odom_pos_x_{0.0f};
  float                                           odom_pos_z_{0.0f};
  float                                           floor_height_m_{0.0f};
  float                                           last_published_height_{0.0f};
  std::shared_ptr<tf2_ros::TransformBroadcaster>  tf_br_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr        imu_sub_;  // LEGACY: E4

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
  PerfAccum    perf_accum_;
  rclcpp::Time last_perf_log_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthObstacleFilterNode>());
  rclcpp::shutdown();
  return 0;
}
