// ─────────────────────────────────────────────────────────────────────────────
// DepthProjectionNode - V2
//
// Agrega alineación gravitacional a la nube de puntos.
// Suscribe:  imagen de profundidad 16-bit + CameraInfo + sensor_msgs/Imu
// Publica:
//   /local_mapper/debug/depth_cloud    — nube cruda (frame óptico)
//   /local_mapper/debug/aligned_cloud  — nube alineada a gravedad
//
// La alineación usa el acelerómetro del IMU integrado en la D435i.
// El ImuFilter aplica low-pass IIR + rechazo de outliers antes de
// pasarlo al GravityAligner, que calcula el cuaternión de corrección.
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

using std::placeholders::_1;

class DepthProjectionNode : public rclcpp::Node {
 public:
  DepthProjectionNode() : rclcpp::Node("depth_projection_node") {
    // ── Parámetros ───────────────────────────────────────────────────────────
    range_min_m_ = static_cast<float>(
        declare_parameter<double>("range_min_m", 0.1));
    range_max_m_ = static_cast<float>(
        declare_parameter<double>("range_max_m", 5.0));

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

    aligned_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/local_mapper/debug/aligned_cloud", 10);

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
    const auto q = aligner_.estimateOrientation(
        imu_filter_->ax(), imu_filter_->ay(), imu_filter_->az());

    geometry_msgs::msg::TransformStamped tf_dyn;
    tf_dyn.header.stamp    = msg->header.stamp;
    tf_dyn.header.frame_id = "gravity_aligned_frame";
    tf_dyn.child_frame_id  = "camera_depth_optical_frame";
    tf_dyn.transform.rotation.w = q.w;
    tf_dyn.transform.rotation.x = q.x;
    tf_dyn.transform.rotation.y = q.y;
    tf_dyn.transform.rotation.z = q.z;
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
        aligned_pub_->get_subscription_count() == 0) return;

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

    // Publicar nube cruda
    if (cloud_pub_->get_subscription_count() > 0) {
      publishCloud(cloud_pub_, msg->header, valid);
    }

    // Publicar nube alineada a gravedad
    if (aligned_pub_->get_subscription_count() > 0) {
      publishAlignedCloud(msg->header, valid);
    }
  }

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
    // Publicar puntos CRUDOS en gravity_aligned_frame (sin rotar).
    // Al estar en el Fixed Frame, RViz no les aplica ninguna TF.
    // Resultado: la nube se ve tal cual la mide la cámara → oscila al
    // inclinar el dispositivo.
    std_msgs::msg::Header raw_header;
    raw_header.stamp    = header.stamp;
    raw_header.frame_id = "gravity_aligned_frame";
    auto pc = makeCloudMsg(raw_header, static_cast<uint32_t>(pts.size()));
    auto* ptr = reinterpret_cast<float*>(pc.data.data());
    for (const auto& pt : pts) { *ptr++ = pt.x; *ptr++ = pt.y; *ptr++ = pt.z; }
    pub->publish(std::move(pc));
  }

  void publishAlignedCloud(
      const std_msgs::msg::Header& header,
      const std::vector<local_mapper::DepthProjector::Point3D>& pts)
  {
    // La nube alineada se publica en camera_depth_optical_frame con los
    // puntos crudos.  La TF gravity_aligned_frame → camera_depth_optical_frame
    // (con q) publicada en onImu() hace que RViz (y tf2_buffer->transform())
    // apliquen q al transformar estos puntos al frame alineado,
    // corrigiéndolos automáticamente.
    std_msgs::msg::Header aligned_header;
    aligned_header.stamp    = header.stamp;
    aligned_header.frame_id = "camera_depth_optical_frame";
    auto pc = makeCloudMsg(aligned_header, static_cast<uint32_t>(pts.size()));
    auto* ptr = reinterpret_cast<float*>(pc.data.data());

    for (const auto& pt : pts) {
      *ptr++ = pt.x; *ptr++ = pt.y; *ptr++ = pt.z;
    }

    aligned_pub_->publish(std::move(pc));
  }

  // ── Miembros ─────────────────────────────────────────────────────────────
  std::unique_ptr<local_mapper::DepthProjector> projector_;
  std::unique_ptr<local_mapper::ImuFilter>      imu_filter_;
  local_mapper::GravityAligner                  aligner_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr        imu_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_pub_;

  float  range_min_m_ = 0.1f;
  float  range_max_m_ = 5.0f;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthProjectionNode>());
  rclcpp::shutdown();
  return 0;
}
