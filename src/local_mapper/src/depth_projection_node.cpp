// ─────────────────────────────────────────────────────────────────────────────
// DepthProjectionNode - V4
//

// Suscribe:  imagen de profundidad 16-bit + CameraInfo
//            + nav_msgs/Odometry (nav_odom) — orientación VIO de nav_odometry
//            + sensor_msgs/Imu  [LEGACY — conservado para Experimento E4]
// Publica:
//   /local_mapper/debug/depth_cloud    — nube cruda (camera_depth_optical_frame, alineada vía TF)
//   /local_mapper/debug/raw_cloud      — nube cruda sin rotar (gravity_aligned_frame, oscila al inclinar)
//   /local_mapper/debug/ground_cloud   — suelo detectado (gravity_aligned_frame)
//   /local_mapper/obstacle_cloud       — obstáculos sobre el suelo (gravity_aligned_frame)
//   /local_mapper/debug/ceiling_cloud  — techo descartado (gravity_aligned_frame)
//
// depth_cloud y raw_cloud tienen los mismos puntos: depth_cloud se ve estático
// en RViz porque la TF dinámica lo alinea; raw_cloud se ve oscilar al inclinar
// la cámara, útil para verificar visualmente que el alineamiento funciona.
//
// GroundEstimator: pipeline de 8 etapas, detecta
// el plano del suelo por RANSAC jerárquico sobre la nube de puntos alineada.
// ─────────────────────────────────────────────────────────────────────────────

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "local_mapper/depth_projector.hpp"
#include "local_mapper/gravity_aligner.hpp"
#include "local_mapper/imu_filter.hpp"
#include "local_mapper/ground_estimator.hpp"

using std::placeholders::_1;

class DepthProjectionNode : public rclcpp::Node {
 public:
  DepthProjectionNode() : rclcpp::Node("depth_projection_node") {
    // ── Parámetros ───────────────────────────────────────────────────────────
    range_min_m_ = static_cast<float>(
        declare_parameter<double>("range_min_m", 0.1));
    range_max_m_ = static_cast<float>(
        declare_parameter<double>("range_max_m", 5.0));

    local_mapper::GroundEstimator::Config ge_cfg;
    ge_cfg.voxel_size_m   = static_cast<float>(
        declare_parameter<double>("voxel_size_m", 0.05));
    ge_cfg.ransac_max_iter = declare_parameter<int>("ransac_max_iter", 100);
    ge_cfg.ransac_inlier_tol = static_cast<float>(
        declare_parameter<double>("ground_inlier_tol", 0.0));
    ge_cfg.min_person_height_m = static_cast<float>(
        declare_parameter<double>("min_person_height_m", 1.3));
    ge_cfg.ceiling_delta_m = static_cast<float>(
        declare_parameter<double>("ceiling_delta_m", 0.1));
    ground_estimator_ = std::make_unique<local_mapper::GroundEstimator>(ge_cfg);

    // ── ImuFilter + GravityAligner [LEGACY] ──────────────────────────────────
    // Conservados para Experimento E4 (comparación con orientación VIO).
    // En operación normal la orientación proviene de onOdom(); estos objetos
    // no se invocan a menos que se reactive el fallback en onImu().
    local_mapper::ImuFilter::Config imu_cfg;
    imu_filter_ = std::make_unique<local_mapper::ImuFilter>(
        imu_cfg, get_logger(), get_clock());

    // ── TF dinámica gravity_aligned_frame → camera_depth_optical_frame ────
    // Se actualiza en onOdom() con la orientación VIO recibida de nav_odometry.
    tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // ── Publicadores ─────────────────────────────────────────────────────────
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/local_mapper/debug/depth_cloud", 10);

    raw_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/local_mapper/debug/raw_cloud", 10);

    ground_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/local_mapper/debug/ground_cloud", 10);

    obstacle_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/local_mapper/obstacle_cloud", 10);

    ceiling_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/local_mapper/debug/ceiling_cloud", 10);

    // ── Suscriptores ─────────────────────────────────────────────────────────
    // El nodo usa nombres genéricos; el launch file remapea al hardware.
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "depth/image", rclcpp::SensorDataQoS(),
        std::bind(&DepthProjectionNode::onDepth, this, _1));

    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "depth/camera_info", rclcpp::SensorDataQoS(),
        std::bind(&DepthProjectionNode::onCameraInfo, this, _1));

    // Suscripción principal: orientación VIO publicada por nav_odometry.
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "nav_odom", rclcpp::QoS(1).reliable(),
        std::bind(&DepthProjectionNode::onOdom, this, _1));

    // [LEGACY] — GravityAligner puro; conservado para Experimento E4.
    // Reactivar descomentando el cuerpo de onImu() y esta suscripción.
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu", rclcpp::SensorDataQoS(),
        std::bind(&DepthProjectionNode::onImu, this, _1));

    RCLCPP_INFO(get_logger(),
        "DepthProjectionNode listo. range=[%.2f, %.2f]m",
        range_min_m_, range_max_m_);
    last_perf_log_ = get_clock()->now();
  }

 private:
  void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (!projector_) {
      projector_ = std::make_unique<local_mapper::DepthProjector>(*msg);
      RCLCPP_INFO(get_logger(),
          "CameraInfo recibida: fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
          projector_->fx(), projector_->fy(),
          projector_->cx(), projector_->cy());
    }
  }

  // ── LEGACY: GravityAligner ───────────────────────────────────────────────
  // Conservado para Experimento E4 (comparación GravityAligner vs VIO).
  // En operación normal la orientación se actualiza en onOdom().
  // Para activar este modo: descomentar el cuerpo y desactivar onOdom.
  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg) {
    (void)msg;  // fallback inactivo; orientación proviene de onOdom()

    // ── LEGACY (descomentar para Experimento E4) ──────────────────────────
    // const auto& a = msg->linear_acceleration;
    // imu_filter_->processAccel(
    //     static_cast<float>(a.x),
    //     static_cast<float>(a.y),
    //     static_cast<float>(a.z));
    // q_ = aligner_.estimateOrientation(
    //     imu_filter_->ax(), imu_filter_->ay(), imu_filter_->az());
    // geometry_msgs::msg::TransformStamped tf_dyn;
    // tf_dyn.header.stamp    = msg->header.stamp;
    // tf_dyn.header.frame_id = "gravity_aligned_frame";
    // tf_dyn.child_frame_id  = "camera_depth_optical_frame";
    // tf_dyn.transform.rotation.w = q_.w;
    // tf_dyn.transform.rotation.x = q_.x;
    // tf_dyn.transform.rotation.y = q_.y;
    // tf_dyn.transform.rotation.z = q_.z;
    // tf_br_->sendTransform(tf_dyn);
  }

  // ── Orientación VIO (fuente principal) ───────────────────────────────────
  // Recibe la odometría de nav_odometry y publica dos TFs:
  //
  //  1) odom → gravity_aligned_frame
  //     Posición XYZ + solo yaw (nivel de burbuja: sigue heading pero
  //     roll=0, pitch=0 siempre).
  //
  //  2) gravity_aligned_frame → camera_depth_optical_frame
  //     Solo roll+pitch (q_rp_, Y-down).
  //
  // Extracción del yaw en Y-down:
  //   Se usa la proyección del vector "adelante" (Z body) sobre el plano
  //   horizontal (XZ en Y-down), en lugar de atan2 sobre el cuaternión.
  //   Esto evita la discontinuidad de signo de atan2 cerca de ±180° que
  //   provoca que q_rp flipee su componente w y los ejes cambien de signo
  //   instantáneamente en RViz.
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // nav_odom se publica en convencion optica nativa (Y abajo, Z adelante):
    // no se necesita conversion de marco al recibirlo.
    q_ = nav_math::Quaternion{
        static_cast<float>(msg->pose.pose.orientation.w),
        static_cast<float>(msg->pose.pose.orientation.x),
        static_cast<float>(msg->pose.pose.orientation.y),
        static_cast<float>(msg->pose.pose.orientation.z)}.normalized();

    // ── Yaw en Y-down: rotación alrededor de +Y (eje vertical) ───────────────
    // Proyectar el vector "adelante" (Z body) al plano horizontal (XZ).
    const nav_math::Vec3 fwd = q_.rotate({0.f, 0.f, 1.f});
    const float fwd_xz = std::sqrt(fwd.x * fwd.x + fwd.z * fwd.z);
    nav_math::Quaternion q_yaw_yd{1.f, 0.f, 0.f, 0.f};  // identidad si cámara vertical
    if (fwd_xz > 1e-4f) {
      const float yaw = std::atan2(fwd.x / fwd_xz, fwd.z / fwd_xz);
      const float hy  = yaw * 0.5f;
      q_yaw_yd = {std::cos(hy), 0.f, std::sin(hy), 0.f};
    }

    // ── Solo roll+pitch (sin yaw) en Y-down ──────────────────────────────────
    // q_rp_ = q_yaw_yd⁻¹ ⊗ q_
    // Signo canónico (w ≥ 0) para evitar flip de ejes en RViz al cruzar ±180°.
    q_rp_ = (q_yaw_yd.conjugate() * q_).normalized();
    if (q_rp_.w < 0.f) {
      q_rp_ = {-q_rp_.w, -q_rp_.x, -q_rp_.y, -q_rp_.z};
    }

    // ── TF 1: odom → gravity_aligned_frame ──────────────────────────────────────────
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

    if (msg->encoding != "16UC1" && msg->encoding != "mono16") {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Encoding inesperado: %s (se espera 16UC1)", msg->encoding.c_str());
      return;
    }

    // Si nadie escucha, no gastar CPU
    if (cloud_pub_->get_subscription_count() == 0 &&
        raw_cloud_pub_->get_subscription_count() == 0 &&
        ground_pub_->get_subscription_count() == 0 &&
        obstacle_pub_->get_subscription_count() == 0 &&
        ceiling_pub_->get_subscription_count() == 0) return;

    // Retroproyectar
    const auto* depth_data = reinterpret_cast<const uint16_t*>(msg->data.data());
    auto points = projector_->projectDepthImage(
        depth_data, msg->width, msg->height);

    // Filtrar puntos fuera de rango
    std::vector<local_mapper::DepthProjector::Point3D> valid;
    valid.reserve(points.size() / 4);
    for (const auto& pt : points) {
      if (std::isnan(pt.z)) continue;
      if (pt.z <= range_min_m_ || pt.z >= range_max_m_) continue;
      valid.push_back(pt);
    }

    // Publicar nube cruda en camera_depth_optical_frame.
    // En RViz (Fixed Frame = gravity_aligned_frame) la TF dinámica
    // publicada en onImu() alinea esta nube automáticamente.
    if (cloud_pub_->get_subscription_count() > 0) {
      publishCloud(cloud_pub_, msg->header, valid);
    }

    // Publicar nube sin rotar en gravity_aligned_frame (debug).
    // Al no aplicarse ninguna TF, oscila visualmente al inclinar la cámara.
    if (raw_cloud_pub_->get_subscription_count() > 0) {
      std_msgs::msg::Header raw_hdr;
      raw_hdr.stamp    = msg->header.stamp;
      raw_hdr.frame_id = "gravity_aligned_frame";
      publishCloud(raw_cloud_pub_, raw_hdr, valid);
    }

    // Estimación del suelo
    if (ground_pub_->get_subscription_count() > 0 ||
        obstacle_pub_->get_subscription_count() > 0 ||
        ceiling_pub_->get_subscription_count() > 0)
    {
      // Convertir a GroundEstimator::Point3D rotando con q_ cacheado de onImu.
      // Los umbrales de altura (min_person_height_m, ceiling_delta_m) requieren
      // coordenadas en gravity_aligned_frame — por eso se rota explícitamente.
      std::vector<local_mapper::GroundEstimator::Point3D> ge_cloud;
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
      if (obstacle_pub_->get_subscription_count() > 0) {
        publishIndexedCloud(obstacle_pub_, aligned_hdr, ge_cloud,
                            ground_estimator_->obstacleIndices());
      }

      // Publicar puntos de techo (CEILING) — para debug/visualización
      if (ceiling_pub_->get_subscription_count() > 0) {
        std::vector<int> ceiling_idx;
        const auto& labels = ground_estimator_->labels();
        for (int i = 0; i < static_cast<int>(labels.size()); ++i)
          if (labels[i] == local_mapper::GroundEstimator::Label::CEILING)
            ceiling_idx.push_back(i);
        publishIndexedCloud(ceiling_pub_, aligned_hdr, ge_cloud, ceiling_idx);
      }

      // Acumular estadísticas de cada frame y emitir resumen cada 10 s
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
        }
        perf_accum_.reset();
        last_perf_log_ = now;
      }
    }
  }  // onDepth

  // ── Helpers de publicación ────────────────────────────────────────────────

  static sensor_msgs::msg::PointCloud2 makeCloudMsg(
      const std_msgs::msg::Header& header, uint32_t n_points)
  {
    sensor_msgs::msg::PointCloud2 pc;
    pc.header = header;
    pc.height = 1;
    pc.width = n_points;
    pc.is_dense = true;
    pc.is_bigendian = false;
    pc.point_step = 12;  // 3 floats × 4 bytes
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
      const std::vector<local_mapper::DepthProjector::Point3D>& pts)
  {
    auto pc = makeCloudMsg(header, static_cast<uint32_t>(pts.size()));
    auto* ptr = reinterpret_cast<float*>(pc.data.data());
    for (const auto& pt : pts) { *ptr++ = pt.x; *ptr++ = pt.y; *ptr++ = pt.z; }
    pub->publish(std::move(pc));
  }

  void publishIndexedCloud(
      const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub,
      const std_msgs::msg::Header& header,
      const std::vector<local_mapper::GroundEstimator::Point3D>& cloud,
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

    void accumulate(const local_mapper::GroundEstimator::PerfStats& p) {
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

  // ── Miembros ─────────────────────────────────────────────────────────────
  std::unique_ptr<local_mapper::DepthProjector>    projector_;
  std::unique_ptr<local_mapper::ImuFilter>         imu_filter_;  // LEGACY: E4
  std::unique_ptr<local_mapper::GroundEstimator>   ground_estimator_;
  local_mapper::GravityAligner                     aligner_;     // LEGACY: E4
  nav_math::Quaternion                              q_{1.0f, 0.0f, 0.0f, 0.0f};
  nav_math::Quaternion                              q_rp_{1.0f, 0.0f, 0.0f, 0.0f};
  std::shared_ptr<tf2_ros::TransformBroadcaster>   tf_br_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr        imu_sub_;  // LEGACY: E4

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ceiling_pub_;

  float  range_min_m_ = 0.1f;
  float  range_max_m_ = 5.0f;
  PerfAccum    perf_accum_;
  rclcpp::Time last_perf_log_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthProjectionNode>());
  rclcpp::shutdown();
  return 0;
}
