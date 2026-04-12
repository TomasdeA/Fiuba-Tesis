// ─────────────────────────────────────────────────────────────────────────────
// DepthProjectionNode - V3
//

// Suscribe:  imagen de profundidad 16-bit + CameraInfo + sensor_msgs/Imu
// Publica:
//   /local_mapper/debug/depth_cloud    — nube cruda (camera_depth_optical_frame, alineada vía TF)
//   /local_mapper/debug/raw_cloud      — nube cruda sin rotar (gravity_aligned_frame, oscila al inclinar)
//   /local_mapper/debug/ground_cloud   — suelo detectado (gravity_aligned_frame)
//   /local_mapper/debug/obstacle_cloud — obstáculos sobre el suelo (gravity_aligned_frame)
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

    // ── ImuFilter ────────────────────────────────────────────────────────────
    local_mapper::ImuFilter::Config imu_cfg;
    imu_filter_ = std::make_unique<local_mapper::ImuFilter>(
        imu_cfg, get_logger(), get_clock());

    // ── TF dinámica gravity_aligned_frame → camera_depth_optical_frame ────
    // Se actualiza en onImu() con el cuaternión q del GravityAligner.
    // q es la rotación mínima que lleva g_medido → (0,-1,0).
    tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // ── Publicadores ─────────────────────────────────────────────────────────
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/local_mapper/debug/depth_cloud", 10);

    raw_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/local_mapper/debug/raw_cloud", 10);

    ground_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/local_mapper/debug/ground_cloud", 10);

    obstacle_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/local_mapper/debug/obstacle_cloud", 10);

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

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu", rclcpp::SensorDataQoS(),
        std::bind(&DepthProjectionNode::onImu, this, _1));

    RCLCPP_INFO(get_logger(),
        "DepthProjectionNode listo. range=[%.2f, %.2f]m",
        range_min_m_, range_max_m_);
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

  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg) {
    const auto& a = msg->linear_acceleration;
    imu_filter_->processAccel(
        static_cast<float>(a.x),
        static_cast<float>(a.y),
        static_cast<float>(a.z));

    // Publicar TF dinámica: gravity_aligned_frame → camera_depth_optical_frame
    // q es la rotación mínima que lleva el vector gravedad medido
    // al vector canónico (0,-1,0).  Como TF padre→hijo, describe
    // la orientación del frame cámara respecto al frame alineado.
    // En RViz (Fixed Frame = gravity_aligned_frame):
    //   - GravityAlignedFrame: ejes quietos, Y apunta "abajo" siempre
    //   - CameraFrame:         ejes rotan al inclinar la cámara
    // q lleva puntos de cámara → alineado (corrige la inclinación).
    // En la convención TF de ROS, q_tf transforma puntos del child al parent,
    // por lo tanto q_tf = q (directo).
    q_ = aligner_.estimateOrientation(
        imu_filter_->ax(), imu_filter_->ay(), imu_filter_->az());

    geometry_msgs::msg::TransformStamped tf_dyn;
    tf_dyn.header.stamp    = msg->header.stamp;
    tf_dyn.header.frame_id = "gravity_aligned_frame";
    tf_dyn.child_frame_id  = "camera_depth_optical_frame";
    tf_dyn.transform.rotation.w = q_.w;
    tf_dyn.transform.rotation.x = q_.x;
    tf_dyn.transform.rotation.y = q_.y;
    tf_dyn.transform.rotation.z = q_.z;
    tf_br_->sendTransform(tf_dyn);
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
        const auto r = q_.rotatePoint(pt.x, pt.y, pt.z);
        ge_cloud.push_back({r[0], r[1], r[2]});
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

      // Log periódico de diagnostico
      if (ground_ok) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
            "Suelo: calidad=%.2f  h=%.3fm  t_total=%.1fms  "
            "voxel=%d/%d  inliers=%d",
            ground_estimator_->groundPlane().quality,
            ground_estimator_->cameraHeightM(),
            ground_estimator_->perfStats().time_total_ms,
            ground_estimator_->perfStats().n_voxel,
            ground_estimator_->perfStats().n_input,
            ground_estimator_->perfStats().n_inliers);
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Suelo no detectado. voxel=%d/%d  obstacle=%zu  ceiling=%zu",
            ground_estimator_->perfStats().n_voxel,
            ground_estimator_->perfStats().n_input,
            ground_estimator_->obstacleIndices().size(),
            [&](){
              size_t n = 0;
              for (const auto& l : ground_estimator_->labels())
                if (l == local_mapper::GroundEstimator::Label::CEILING) ++n;
              return n;
            }());
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

  // ── Miembros ─────────────────────────────────────────────────────────────
  std::unique_ptr<local_mapper::DepthProjector>    projector_;
  std::unique_ptr<local_mapper::ImuFilter>         imu_filter_;
  std::unique_ptr<local_mapper::GroundEstimator>   ground_estimator_;
  local_mapper::GravityAligner                     aligner_;
  local_mapper::GravityAligner::Quaternion         q_{1.0f, 0.0f, 0.0f, 0.0f};
  std::shared_ptr<tf2_ros::TransformBroadcaster>   tf_br_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr        imu_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ceiling_pub_;

  float  range_min_m_ = 0.1f;
  float  range_max_m_ = 5.0f;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthProjectionNode>());
  rclcpp::shutdown();
  return 0;
}
