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
#include <limits>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "local_mapper/depth_projector.hpp"
#include "local_mapper/gravity_aligner.hpp"
#include "local_mapper/imu_filter.hpp"
#include "local_mapper/ground_estimator.hpp"
#include "local_mapper/occupancy_mapper.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>

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

    // ── OccupancyMapper ──────────────────────────────────────────────────────
    local_mapper::OccupancyMapper::Config om_cfg;
    om_cfg.cell_size_m       = static_cast<float>(
        declare_parameter<double>("occupancy_cell_size_m", 0.10));
    om_cfg.grid_size         = declare_parameter<int>("occupancy_grid_size", 200);
    om_cfg.l_occ             = static_cast<float>(
        declare_parameter<double>("occupancy_l_occ",  2.197));
    om_cfg.l_free            = static_cast<float>(
        declare_parameter<double>("occupancy_l_free", -0.847));
    om_cfg.l_min             = static_cast<float>(
        declare_parameter<double>("occupancy_l_min", -5.0));
    om_cfg.l_max             = static_cast<float>(
        declare_parameter<double>("occupancy_l_max",  5.0));
    om_cfg.max_range_m       = static_cast<float>(
        declare_parameter<double>("occupancy_max_range_m", 5.0));
    om_cfg.forget_radius_m   = static_cast<float>(
        declare_parameter<double>("occupancy_forget_radius_m", 5.0));
    om_cfg.enable_raycasting = declare_parameter<bool>("occupancy_enable_raycasting", true);
    occupancy_mapper_ = std::make_unique<local_mapper::OccupancyMapper>(om_cfg);

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

    // Altura de la cámara sobre el suelo (calibración).
    // Se publica por única vez salvo que el nuevo estimado difiera > 10 cm
    // del último publicado (indica que la medición anterior fue errónea).
    camera_height_pub_ = create_publisher<std_msgs::msg::Float32>(
        "/local_mapper/camera_height", rclcpp::QoS(1).transient_local());

    // Mapa de ocupación 2D acumulado en el marco odom.
    occupancy_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/local_mapper/occupancy_grid", rclcpp::QoS(1).transient_local());

    // ── Suscriptores ─────────────────────────────────────────────────────────
    // El nodo usa nombres genéricos; el launch file remapea al hardware.
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "depth/image", rclcpp::SensorDataQoS(),
        std::bind(&DepthProjectionNode::onDepth, this, _1));

    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "depth/camera_info", rclcpp::SensorDataQoS(),
        std::bind(&DepthProjectionNode::onCameraInfo, this, _1));

    // Suscripción principal: orientación VIO publicada por nav_odometry.
    // La posición recibida se usa también por OccupancyMapper para registrar
    // el mapa al frame odom.
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

    // ── Guardar posición odom para OccupancyMapper ──────────────────────────────
    odom_pos_x_ = static_cast<float>(msg->pose.pose.position.x);
    odom_pos_z_ = static_cast<float>(msg->pose.pose.position.z);
    q_yaw_yd_cached_ = q_yaw_yd;

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

    if (msg->encoding != "16UC1") {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Encoding inesperado: %s (se espera 16UC1)", msg->encoding.c_str());
      return;
    }

    // Si nadie escucha, no gastar CPU
    if (cloud_pub_->get_subscription_count() == 0 &&
        raw_cloud_pub_->get_subscription_count() == 0 &&
        ground_pub_->get_subscription_count() == 0 &&
        obstacle_pub_->get_subscription_count() == 0 &&
        ceiling_pub_->get_subscription_count() == 0 &&
        occupancy_pub_->get_subscription_count() == 0) return;

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
        ceiling_pub_->get_subscription_count() > 0 ||
        occupancy_pub_->get_subscription_count() > 0)
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

      // ── Mapa de ocupación (frame odom, absoluto) ─────────────
      // El grid vive en frame odom. Los obstáculos se insertan en coordenadas
      // odom para que el grid no necesite rotar cuando la cámara gira en yaw.
      // El shift usa d_odom directamente; shift() actualiza origin_x_/z_.
      {
        // ── 1. Shift: desplazar el grid según el movimiento en odom ───────────
        if (std::isnan(prev_odom_pos_x_)) {
          RCLCPP_WARN_ONCE(get_logger(),
              "Primer frame de occupancy — posición anterior no disponible, "
              "shift omitido. Si este mensaje no aparece solo una vez, "
              "nav_odometry podría no estar publicando.");
        } else {
          // Desplazamiento de la cámara en odom. El grid se desplaza en
          // dirección opuesta: si la cámara avanza +Z, los obstáculos
          // históricos retroceden en la ventana → shift_cj negativo.
          const nav_math::Vec3 d_odom{
              odom_pos_x_ - prev_odom_pos_x_, 0.f,
              odom_pos_z_ - prev_odom_pos_z_};
          // Acumular desplazamiento sub-celda. round() sobre el delta de un
          // solo frame perdería fracciones de celda a velocidades normales.
          const float cellSizeM = occupancy_mapper_->cellSizeM();
          shift_accum_ci_ += -d_odom.x / cellSizeM;
          shift_accum_cj_ += -d_odom.z / cellSizeM;
          const int shift_ci = static_cast<int>(shift_accum_ci_);
          const int shift_cj = static_cast<int>(shift_accum_cj_);
          shift_accum_ci_ -= static_cast<float>(shift_ci);
          shift_accum_cj_ -= static_cast<float>(shift_cj);

          RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
              " odom=(%.3f, %.3f)  d_odom=(%.4f, %.4f)  shift=(%d, %d)",
              odom_pos_x_, odom_pos_z_,
              d_odom.x, d_odom.z,
              shift_ci, shift_cj);

          if (d_odom.x == 0.f && d_odom.z == 0.f) {
            RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000,
                " Traslación nula en odom — nav_odometry podría no estar "
                "estimando movimiento (¿VIO sin feature tracks?). "
                "La rejilla no se desplazará y el mapa solo mostrará el FOV actual.");
          }

          if (std::abs(shift_ci) > 50 || std::abs(shift_cj) > 50) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                " Shift anormalmente grande (%d, %d) — posible salto en "
                "odometría o error de convenio de ejes. d_odom=(%.3f, %.3f).",
                shift_ci, shift_cj, d_odom.x, d_odom.z);
          }

          occupancy_mapper_->shift(shift_ci, shift_cj);
        }
        prev_odom_pos_x_ = odom_pos_x_;
        prev_odom_pos_z_ = odom_pos_z_;

        // ── 2. Actualizar altura del piso para la visualización ───────────────
        if (ground_ok) {
          const float h = ground_estimator_->cameraHeightM();
          if (h > 0.5f && h < 2.5f) floor_height_m_ = h;
        }

        // ── 3. Registrar obstáculos en coordenadas odom absolutas ────────────
        // Los puntos de ge_cloud están en gravity_aligned_frame (XZ relativo
        // a la cámara, con roll/pitch corregido). Para llevarlos a odom:
        //   p_odom = q_yaw ⊗ p_gaf + {odom_pos_x_, 0, odom_pos_z_}
        const auto& obs_idx = ground_estimator_->obstacleIndices();
        std::vector<local_mapper::OccupancyMapper::Point2D> occ_pts;
        occ_pts.reserve(obs_idx.size());
        for (int idx : obs_idx) {
          const auto& p = ge_cloud[idx];
          const nav_math::Vec3 p_odom =
              q_yaw_yd_cached_.rotate({p.x, 0.f, p.z});
          occ_pts.push_back({p_odom.x + odom_pos_x_, p_odom.z + odom_pos_z_});
        }
        occupancy_mapper_->update(occ_pts, odom_pos_x_, odom_pos_z_);

        // ── 4. Publicar OccupancyGrid en frame odom (~2 Hz, throttled) ───────
        // Publicar a 30 Hz llena la queue de RViz → drops continuos.
        // A 30 fps de cámara, publicar cada 15 frames = ~2 Hz.
        ++occ_frame_count_;
        if (occupancy_pub_->get_subscription_count() > 0 &&
            occ_frame_count_ >= 15) {
          occ_frame_count_ = 0;
          nav_msgs::msg::OccupancyGrid occ_msg;
          occ_msg.header.stamp    = msg->header.stamp;
          occ_msg.header.frame_id = "odom";
          occ_msg.info.map_load_time = msg->header.stamp;
          occ_msg.info.resolution = occupancy_mapper_->cellSizeM();
          const int gs = occupancy_mapper_->gridSize();
          occ_msg.info.width  = static_cast<uint32_t>(gs);
          occ_msg.info.height = static_cast<uint32_t>(gs);
          // El vértice (0,0) del grid en odom. origin.y = altura del suelo.
          // kHalfSqrt2 rota +90° en X para que el plano 2D XY del grid quede
          // alineado con el plano XZ de odom (Y=abajo en convención óptica).
          static constexpr double kHalfSqrt2 = 0.7071067811865476;
          occ_msg.info.origin.position.x = static_cast<double>(occupancy_mapper_->originX());
          occ_msg.info.origin.position.y = static_cast<double>(floor_height_m_);
          occ_msg.info.origin.position.z = static_cast<double>(occupancy_mapper_->originZ());
          occ_msg.info.origin.orientation.w = kHalfSqrt2;
          occ_msg.info.origin.orientation.x = kHalfSqrt2;
          occ_msg.info.origin.orientation.y = 0.0;
          occ_msg.info.origin.orientation.z = 0.0;

          const auto& log_odds = occupancy_mapper_->logOdds();
          occ_msg.data.resize(log_odds.size());
          for (std::size_t k = 0; k < log_odds.size(); ++k) {
            const int ci = static_cast<int>(k) / gs;
            const int cj = static_cast<int>(k) % gs;
            occ_msg.data[ci + cj * gs] =
                local_mapper::OccupancyMapper::logOddsToNavMsg(log_odds[k]);
          }
          occupancy_pub_->publish(std::move(occ_msg));
        }
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

      // ── Publicación de altura de cámara (calibración) ─────────────────────
      // Solo se publica si el suelo fue detectado, la calidad es aceptable
      // y el nuevo valor difiere en más de 10 cm del último publicado.
      if (ground_ok && camera_height_pub_->get_subscription_count() > 0) {
        const float h = ground_estimator_->cameraHeightM();
        if (h > 0.5f && h < 2.5f) {  // rango plausible de altura humana
          const float delta = std::abs(h - last_published_height_);
          if (delta > 0.10f) {  // cambio mayor a 10 cm → publicar nueva calibración
            std_msgs::msg::Float32 height_msg;
            height_msg.data = h;
            camera_height_pub_->publish(height_msg);
            RCLCPP_INFO(get_logger(),
                "[camera_height] %.3f m publicado (delta=%.3f m desde el anterior)",
                h, delta);
            last_published_height_ = h;
          }
        }
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
  std::unique_ptr<local_mapper::OccupancyMapper>   occupancy_mapper_;
  local_mapper::GravityAligner                     aligner_;     // LEGACY: E4
  nav_math::Quaternion                              q_{1.0f, 0.0f, 0.0f, 0.0f};
  nav_math::Quaternion                              q_rp_{1.0f, 0.0f, 0.0f, 0.0f};
  nav_math::Quaternion                              q_yaw_yd_cached_{1.0f, 0.0f, 0.0f, 0.0f};
  float                                             odom_pos_x_{0.0f};
  float                                             odom_pos_z_{0.0f};
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
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr         camera_height_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr   occupancy_pub_;

  // Última altura publicada (inicializado a 0 para forzar la primera publicación)
  float last_published_height_ = 0.0f;

  // Altura estimada del piso en el marco odom (Y abajo → Y positivo = bajo la cámara).
  // Se actualiza cada frame cuando ground_ok es true y h está en rango plausible.
  // Se usa para colocar el OccupancyGrid a la altura real del suelo en RViz.
  float floor_height_m_ = 0.0f;

  // Posición de la cámara en el frame odom del frame anterior.
  // Se inicializa a NaN para detectar el primer frame (sin shift).
  float prev_odom_pos_x_ = std::numeric_limits<float>::quiet_NaN();
  float prev_odom_pos_z_ = std::numeric_limits<float>::quiet_NaN();

  // Acumuladores de desplazamiento sub-celda para el shift del OccupancyMapper.
  // A 30 fps y velocidad normal (~1.2 m/s), el movimiento por frame es ~0.04 m
  // = 0.4 celdas. std::round(0.4) = 0 → sin acumulación el grid nunca se
  // desplazaría. Al acumular la fracción entre frames se garantiza que cada
  // metro recorrido produce exactamente 10 celdas de shift.
  float shift_accum_ci_ = 0.f;
  float shift_accum_cj_ = 0.f;

  // Contador de frames para throttle de OccupancyGrid (~5 Hz = cada 6 frames a 30 fps).
  int occ_frame_count_ = 0;

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
