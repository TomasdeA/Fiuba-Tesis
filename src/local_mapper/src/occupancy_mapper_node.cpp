// ─────────────────────────────────────────────────────────────────────────────
// OccupancyMapperNode  (local_mapper)
//
// Construye un mapa de ocupación 2D en log-odds a partir de la nube de
// obstáculos publicada por depth_obstacle_filter.
//
// Suscribe (tres topics sincronizados; mismo stamp por frame de profundidad):
//   /depth_obstacle_filter/obstacle_cloud_odom — obstáculos en frame odom (PointCloud2)
//   /depth_obstacle_filter/free_endpoints  — endpoints de rayos libres en odom
//   /depth_obstacle_filter/sensor_pos      — posición de la cámara en odom
//                                            (PointStamped; point.y = floor_height_m)
//
// Publica:
//   /local_mapper/occupancy_grid  — nav_msgs/OccupancyGrid en frame "odom"
//
// Sincronización perfecta:
//   Los tres topics de entrada se publican desde el mismo callback en
//   depth_obstacle_filter y comparten el mismo header.stamp (= stamp del frame
//   de profundidad). message_filters::ExactTimeSynchronizer garantiza que
//   onSync() solo se invoca cuando los tres mensajes del mismo instante están
//   disponibles: el mapa se actualiza con datos estrictamente coherentes.
//
// Formato de las nubes de entrada:
//   PointCloud2 unordered (height=1), puntos de 12 bytes (float32 x, y, z).
//   Solo se usan x y z (componentes XZ del frame odom) para el mapa 2D.
// ─────────────────────────────────────────────────────────────────────────────

#include <rclcpp/rclcpp.hpp>
#include <limits>
#include <cmath>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include "local_mapper/occupancy_mapper.hpp"

using Cloud = sensor_msgs::msg::PointCloud2;
using Point = geometry_msgs::msg::PointStamped;
using SyncPolicy =
    message_filters::sync_policies::ExactTime<Cloud, Cloud, Point>;
using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

class OccupancyMapperNode : public rclcpp::Node {
 public:
  OccupancyMapperNode() : rclcpp::Node("local_mapper") {
    // ── Parámetros ───────────────────────────────────────────────────────────
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

    publish_every_n_frames_ = declare_parameter<int>("occupancy_publish_every_n_frames", 15);

    // ── Publicador ───────────────────────────────────────────────────────────
    occupancy_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/local_mapper/occupancy_grid", rclcpp::QoS(1).transient_local());

    // ── Sincronizador exacto sobre los tres topics de depth_obstacle_filter ──
    // Los tres topics comparten siempre el mismo stamp (mismo depth frame),
    // por lo que ExactTimeSynchronizer nunca descarta mensajes por desalineación.
    constexpr int kQueueDepth = 10;
    obstacle_sub_.subscribe(this, "/depth_obstacle_filter/obstacle_cloud_odom",
                            rclcpp::QoS(kQueueDepth).get_rmw_qos_profile());
    free_sub_.subscribe(this, "/depth_obstacle_filter/free_endpoints",
                        rclcpp::QoS(kQueueDepth).get_rmw_qos_profile());
    sensor_pos_sub_.subscribe(this, "/depth_obstacle_filter/sensor_pos",
                              rclcpp::QoS(kQueueDepth).get_rmw_qos_profile());

    sync_ = std::make_shared<Synchronizer>(
        SyncPolicy(kQueueDepth),
        obstacle_sub_, free_sub_, sensor_pos_sub_);
    sync_->registerCallback(
        std::bind(&OccupancyMapperNode::onSync, this,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  std::placeholders::_3));

    RCLCPP_INFO(get_logger(),
        "OccupancyMapperNode listo. grid=%dx%d @ %.2fm/celda. "
        "Publicando OccupancyGrid cada %d frames.",
        om_cfg.grid_size, om_cfg.grid_size,
        om_cfg.cell_size_m, publish_every_n_frames_);
  }

 private:
  // ── Callback principal (ExactTimeSynchronizer) ───────────────────────────
  void onSync(
      const Cloud::ConstSharedPtr& obstacle_cloud,
      const Cloud::ConstSharedPtr& free_endpoints,
      const Point::ConstSharedPtr& sensor_pos)
  {
    const float sx = static_cast<float>(sensor_pos->point.x);
    const float sz = static_cast<float>(sensor_pos->point.z);
    const float floor_height_m = static_cast<float>(sensor_pos->point.y);

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
    auto decodeCloud = [](const Cloud& cloud)
        -> std::vector<local_mapper::OccupancyMapper::Point2D>
    {
      std::vector<local_mapper::OccupancyMapper::Point2D> pts;
      pts.reserve(cloud.width);
      const auto* data = reinterpret_cast<const float*>(cloud.data.data());
      for (uint32_t i = 0; i < cloud.width; ++i) {
        pts.push_back({data[i * 3], data[i * 3 + 2]});  // x, z (skip y)
      }
      return pts;
    };

    const auto occ_pts  = decodeCloud(*obstacle_cloud);
    const auto free_pts = decodeCloud(*free_endpoints);

    // ── Actualizar mapa ───────────────────────────────────────────────────────
    occupancy_mapper_->update(occ_pts, sx, sz, free_pts);

    // ── Publicar OccupancyGrid (throttled para no saturar RViz) ──────────────
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
      static constexpr double kHalfSqrt2 = 0.7071067811865476;
      occ_msg.info.origin.position.x = static_cast<double>(occupancy_mapper_->originX());
      occ_msg.info.origin.position.y = static_cast<double>(floor_height_m);
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

  // ── Miembros ──────────────────────────────────────────────────────────────
  std::unique_ptr<local_mapper::OccupancyMapper> occupancy_mapper_;

  message_filters::Subscriber<Cloud> obstacle_sub_;
  message_filters::Subscriber<Cloud> free_sub_;
  message_filters::Subscriber<Point> sensor_pos_sub_;
  std::shared_ptr<Synchronizer>      sync_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_pub_;

  // Posición del sensor del frame anterior (para shift del grid)
  float prev_sx_ = std::numeric_limits<float>::quiet_NaN();
  float prev_sz_ = std::numeric_limits<float>::quiet_NaN();

  // Acumuladores de desplazamiento sub-celda
  float shift_accum_ci_ = 0.f;
  float shift_accum_cj_ = 0.f;

  int occ_frame_count_         = 0;
  int publish_every_n_frames_  = 15;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccupancyMapperNode>());
  rclcpp::shutdown();
  return 0;
}
