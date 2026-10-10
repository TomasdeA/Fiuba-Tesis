// ─────────────────────────────────────────────────────────────────────────────
// OccupancyMapperNode  (local_mapper)
//
// Construye un mapa de ocupación 2D en log-odds a partir de la nube de
// obstáculos publicada por depth_obstacle_filter.
//
// Suscribe:
//   /depth_obstacle_filter/obstacle_cloud  — obstáculos en gravity_aligned_frame
//   /depth_obstacle_filter/ground_evidence — suelo observado en gravity_aligned_frame
//   nav_odom                               — pose usada para llevar datos a odom
//   /depth_obstacle_filter/camera_height   — altura de piso para RViz
//
// Publica:
//   /local_mapper/occupancy_grid  — nav_msgs/OccupancyGrid en frame "odom"
//
// Sincronización:
//   obstacle_cloud y ground_evidence se sincronizan por stamp del frame de
//   profundidad. Se espera la pose exacta o dos poses cercanas que encierren
//   el sello de la nube; se interpola posición/yaw, sin extrapolar.
//
// Formato de las nubes de entrada:
//   PointCloud2 unordered (height=1), puntos de 12 bytes (float32 x, y, z).
//   Solo se usan x y z (componentes XZ) para el mapa 2D.
// ─────────────────────────────────────────────────────────────────────────────

#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <deque>
#include <chrono>
#include <nav_math/nav_math.hpp>
#include <limits>
#include <cmath>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include "local_mapper/occupancy_mapper.hpp"
#include "local_mapper/timed_pose_buffer.hpp"

using Cloud = sensor_msgs::msg::PointCloud2;
using SyncPolicy =
    message_filters::sync_policies::ExactTime<Cloud, Cloud>;
using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

class OccupancyMapperNode : public rclcpp::Node {
 public:
  OccupancyMapperNode() : rclcpp::Node("local_mapper") {
    // ── Parámetros ───────────────────────────────────────────────────────────
    local_mapper::OccupancyMapper::Config om_cfg;
    om_cfg.cell_size_m       = static_cast<float>(
        declare_parameter<double>("occupancy_cell_size_m", 0.10));
    om_cfg.grid_size         = declare_parameter<int>("occupancy_grid_size", 100);
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
    om_cfg.raycast_angular_bins =
        declare_parameter<int>("occupancy_raycast_angular_bins", 360);
    occupancy_mapper_ = std::make_unique<local_mapper::OccupancyMapper>(om_cfg);

    odom_wait_s_ = declare_parameter<double>("occupancy_odom_wait_s", 0.35);
    const double gap_s = declare_parameter<double>("occupancy_odom_max_gap_s", 0.20);
    if (!std::isfinite(odom_wait_s_) || odom_wait_s_ <= 0.0 ||
        !std::isfinite(gap_s) || gap_s <= 0.0) {
      throw std::invalid_argument("odometry wait and gap must be finite and positive");
    }
    odom_max_gap_ns_ = static_cast<int64_t>(gap_s * 1e9);
    pending_timer_ = create_wall_timer(std::chrono::milliseconds(20),
        std::bind(&OccupancyMapperNode::drainPending, this));

    odom_source_ = declare_parameter<std::string>("odom_source", "nav_odom");
    if (odom_source_ != "nav_odom" && odom_source_ != "rtabmap_odom") {
      RCLCPP_WARN(get_logger(),
          "odom_source='%s' invalido; usando nav_odom",
          odom_source_.c_str());
      odom_source_ = "nav_odom";
    }

    publish_every_n_frames_ =
        std::max(1, static_cast<int>(declare_parameter<int>(
                        "occupancy_publish_every_n_frames", 1)));

    // ── Publicador ───────────────────────────────────────────────────────────
    occupancy_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/local_mapper/occupancy_grid", rclcpp::QoS(1).transient_local());
    tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // ── Sincronizador exacto sobre topics locales de depth_obstacle_filter ──
    // Ambos topics comparten siempre el mismo stamp (mismo depth frame),
    // por lo que ExactTimeSynchronizer nunca descarta mensajes por desalineación.
    constexpr int kQueueDepth = 10;
    obstacle_sub_.subscribe(this, "/depth_obstacle_filter/obstacle_cloud",
                            rclcpp::QoS(kQueueDepth).get_rmw_qos_profile());
    ground_sub_.subscribe(this, "/depth_obstacle_filter/ground_evidence",
                        rclcpp::QoS(kQueueDepth).get_rmw_qos_profile());

    sync_ = std::make_shared<Synchronizer>(
        SyncPolicy(kQueueDepth),
        obstacle_sub_, ground_sub_);
    sync_->registerCallback(
        std::bind(&OccupancyMapperNode::onSync, this,
                  std::placeholders::_1,
                  std::placeholders::_2));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "nav_odom", rclcpp::QoS(20).reliable(),
        std::bind(&OccupancyMapperNode::onOdom, this, std::placeholders::_1));

    camera_height_sub_ = create_subscription<std_msgs::msg::Float32>(
        "/depth_obstacle_filter/camera_height",
        rclcpp::QoS(1).transient_local(),
        [this](const std_msgs::msg::Float32::SharedPtr msg) {
          floor_height_m_ = msg->data;
        });

    RCLCPP_INFO(get_logger(),
        "OccupancyMapperNode listo. grid=%dx%d @ %.2fm/celda odom_source=%s. "
        "Publicando OccupancyGrid cada %d frames.",
        om_cfg.grid_size, om_cfg.grid_size,
        om_cfg.cell_size_m, odom_source_.c_str(), publish_every_n_frames_);
  }

 private:
  struct OdomSample {
    rclcpp::Time stamp;
    float x = 0.f;
    float z = 0.f;
    nav_math::Quaternion yaw{1.f, 0.f, 0.f, 0.f};
  };

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    const nav_math::Quaternion q_msg = nav_math::Quaternion{
        static_cast<float>(msg->pose.pose.orientation.w),
        static_cast<float>(msg->pose.pose.orientation.x),
        static_cast<float>(msg->pose.pose.orientation.y),
        static_cast<float>(msg->pose.pose.orientation.z)}.normalized();

    float pos_x = static_cast<float>(msg->pose.pose.position.x);
    float pos_z = static_cast<float>(msg->pose.pose.position.z);
    nav_math::Quaternion q_odom{1.f, 0.f, 0.f, 0.f};

    if (odom_source_ == "rtabmap_odom") {
      static const nav_math::Quaternion q_ros_to_internal{
          0.5f, 0.5f, -0.5f, 0.5f};
      q_odom = (q_ros_to_internal * q_msg).normalized();
      pos_x = -static_cast<float>(msg->pose.pose.position.y);
      pos_z =  static_cast<float>(msg->pose.pose.position.x);
    } else {
      q_odom = q_msg;
    }

    const nav_math::Vec3 fwd = q_odom.rotate({0.f, 0.f, 1.f});
    const float fwd_xz = std::sqrt(fwd.x * fwd.x + fwd.z * fwd.z);
    nav_math::Quaternion q_yaw{1.f, 0.f, 0.f, 0.f};
    if (fwd_xz > 1e-4f) {
      const float yaw = std::atan2(fwd.x / fwd_xz, fwd.z / fwd_xz);
      const float hy = yaw * 0.5f;
      q_yaw = {std::cos(hy), 0.f, std::sin(hy), 0.f};
    }

    odom_buffer_.insert({rclcpp::Time(msg->header.stamp).nanoseconds(), pos_x, pos_z,
        2.0 * std::atan2(q_yaw.y, q_yaw.w)});
    drainPending();
    publishGravityAlignedTf(msg->header.stamp, pos_x, pos_z, q_yaw);
  }

  void publishGravityAlignedTf(
      const builtin_interfaces::msg::Time& stamp,
      float pos_x,
      float pos_z,
      const nav_math::Quaternion& q_yaw)
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = "gravity_aligned_frame";
    tf_msg.transform.translation.x = pos_x;
    tf_msg.transform.translation.y = 0.0;
    tf_msg.transform.translation.z = pos_z;
    tf_msg.transform.rotation.w = q_yaw.w;
    tf_msg.transform.rotation.x = q_yaw.x;
    tf_msg.transform.rotation.y = q_yaw.y;
    tf_msg.transform.rotation.z = q_yaw.z;
    tf_br_->sendTransform(tf_msg);
  }

  struct PendingFrame {
    Cloud::ConstSharedPtr obstacle, ground;
    std::chrono::steady_clock::time_point received;
  };

  void onSync(const Cloud::ConstSharedPtr& obstacle,
              const Cloud::ConstSharedPtr& ground) {
    const auto stamp = rclcpp::Time(obstacle->header.stamp).nanoseconds();
    // Keep integration chronological; a late frame must not move the map back.
    if (stamp <= last_processed_ns_) return;
    auto it = std::lower_bound(pending_.begin(), pending_.end(), stamp,
        [](const PendingFrame& p, int64_t t) {
          return rclcpp::Time(p.obstacle->header.stamp).nanoseconds() < t;
        });
    if (it != pending_.end() &&
        rclcpp::Time(it->obstacle->header.stamp).nanoseconds() == stamp) return;
    pending_.insert(it, {obstacle, ground, std::chrono::steady_clock::now()});
    while (pending_.size() > 30) {
      pending_.pop_front();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Cola temporal llena: se descarta la observacion mas antigua");
    }
    drainPending();
  }

  void drainPending() {
    while (!pending_.empty()) {
      const auto& frame = pending_.front();
      const int64_t stamp = rclcpp::Time(frame.obstacle->header.stamp).nanoseconds();
      local_mapper::TimedPoseBuffer::Pose pose{};
      const auto status = odom_buffer_.lookup(stamp, odom_max_gap_ns_, pose);
      const double waited = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - frame.received).count();
      if (status != local_mapper::TimedPoseBuffer::Status::Ready) {
        if (status == local_mapper::TimedPoseBuffer::Status::Pending &&
            waited < odom_wait_s_) return;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Sin pose temporal valida: se descarta observacion (estado=%d, espera=%.3fs)",
            static_cast<int>(status), waited);
        last_processed_ns_ = stamp;
        pending_.pop_front();
        continue;
      }
      OdomSample odom;
      odom.stamp = rclcpp::Time(frame.obstacle->header.stamp);
      odom.x = static_cast<float>(pose.x);
      odom.z = static_cast<float>(pose.z);
      odom.yaw = {static_cast<float>(std::cos(pose.yaw*.5)), 0.f,
                  static_cast<float>(std::sin(pose.yaw*.5)), 0.f};
      integrateFrame(frame.obstacle, frame.ground, odom);
      last_processed_ns_ = stamp;
      pending_.pop_front();
    }
  }

  void integrateFrame(const Cloud::ConstSharedPtr& obstacle_cloud,
                      const Cloud::ConstSharedPtr& ground_evidence,
                      const OdomSample& odom) {
    const float sx = odom.x;
    const float sz = odom.z;

    // ── Desplazamiento del grid (cámara movida desde el frame anterior) ──────
    // shift_accum acumula fracciones de celda para evitar pérdida de resolución
    // a velocidades bajas (ej. 1.2 m/s @ 30 Hz → 0.04 m/frame = 0.4 celdas).
    if (!std::isnan(prev_sx_)) {
      const float cellSizeM = occupancy_mapper_->cellSizeM();
      shift_accum_ci_ += -(sx - prev_sx_) / cellSizeM;
      shift_accum_cj_ += -(sz - prev_sz_) / cellSizeM;
      const int shift_ci = static_cast<int>(shift_accum_ci_);
      const int shift_cj = static_cast<int>(shift_accum_cj_);
      shift_accum_ci_ -= static_cast<float>(shift_ci);
      shift_accum_cj_ -= static_cast<float>(shift_cj);

      if (std::abs(shift_ci) > 50 || std::abs(shift_cj) > 50) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Shift anormalmente grande (%d, %d) — posible salto en odometría. "
            "d=(%.3f, %.3f)m",
            shift_ci, shift_cj, sx - prev_sx_, sz - prev_sz_);
      }

      occupancy_mapper_->shift(shift_ci, shift_cj);
    }
    prev_sx_ = sx;
    prev_sz_ = sz;

    // ── Decodificar PointCloud2 → Point2D ────────────────────────────────────
    // Formato garantizado: height=1, point_step=12, campos float32 x/y/z.
    // Solo se usan x y z (XZ en frame odom = plano horizontal del mapa 2D).
    auto decodeCloud = [&odom](const Cloud& cloud)
        -> std::vector<local_mapper::OccupancyMapper::Point2D>
    {
      std::vector<local_mapper::OccupancyMapper::Point2D> pts;
      pts.reserve(cloud.width);
      const auto* data = reinterpret_cast<const float*>(cloud.data.data());
      for (uint32_t i = 0; i < cloud.width; ++i) {
        const nav_math::Vec3 local{data[i * 3], 0.f, data[i * 3 + 2]};
        const nav_math::Vec3 p_odom = odom.yaw.rotate(local);
        pts.push_back({p_odom.x + odom.x, p_odom.z + odom.z});
      }
      return pts;
    };

    const auto occ_pts  = decodeCloud(*obstacle_cloud);
    const auto ground_pts = decodeCloud(*ground_evidence);

    // ── Actualizar mapa ──────────────────────────────────────────────────────
    occupancy_mapper_->update(occ_pts, sx, sz, ground_pts);

    // ── Publicar OccupancyGrid con la cadencia configurada ───────────────────
    ++occ_frame_count_;
    if (occupancy_pub_->get_subscription_count() > 0 &&
        occ_frame_count_ >= publish_every_n_frames_) {
      occ_frame_count_ = 0;

      nav_msgs::msg::OccupancyGrid occ_msg;
      occ_msg.header.stamp    = obstacle_cloud->header.stamp;
      occ_msg.header.frame_id = "odom";
      occ_msg.info.map_load_time = obstacle_cloud->header.stamp;
      occ_msg.info.resolution = occupancy_mapper_->cellSizeM();
      const int gs = occupancy_mapper_->gridSize();
      occ_msg.info.width  = static_cast<uint32_t>(gs);
      occ_msg.info.height = static_cast<uint32_t>(gs);

      // El vértice (0,0) de la rejilla en frame odom.
      // origin.y = altura del suelo publicada por depth_obstacle_filter.
      // La rotación de +90° en X alinea el plano XY del grid con el plano XZ
      // de odom (Y=abajo en convención óptica).
      occ_msg.info.origin.position.x = static_cast<double>(occupancy_mapper_->originX());
      occ_msg.info.origin.position.y = static_cast<double>(floor_height_m_);
      occ_msg.info.origin.position.z = static_cast<double>(occupancy_mapper_->originZ());
      occ_msg.info.origin.orientation.w = nav_math::kHalfSqrt2;
      occ_msg.info.origin.orientation.x = nav_math::kHalfSqrt2;
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

  // ── Miembros ──────────────────────────────────────────────────────────────
  std::unique_ptr<local_mapper::OccupancyMapper> occupancy_mapper_;

  message_filters::Subscriber<Cloud> obstacle_sub_;
  message_filters::Subscriber<Cloud> ground_sub_;
  std::shared_ptr<Synchronizer>      sync_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr camera_height_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_;

  // Posición del sensor del frame anterior (para shift del grid)
  float prev_sx_ = std::numeric_limits<float>::quiet_NaN();
  float prev_sz_ = std::numeric_limits<float>::quiet_NaN();

  // Acumuladores de desplazamiento sub-celda
  float shift_accum_ci_ = 0.f;
  float shift_accum_cj_ = 0.f;
  float floor_height_m_ = 0.f;
  local_mapper::TimedPoseBuffer odom_buffer_;
  std::deque<PendingFrame> pending_;
  rclcpp::TimerBase::SharedPtr pending_timer_;
  double odom_wait_s_ = 0.35;
  int64_t odom_max_gap_ns_ = 200000000;
  int64_t last_processed_ns_ = std::numeric_limits<int64_t>::min();
  std::string odom_source_ = "nav_odom";

  int occ_frame_count_        = 0;
  int publish_every_n_frames_ = 1;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccupancyMapperNode>());
  rclcpp::shutdown();
  return 0;
}
