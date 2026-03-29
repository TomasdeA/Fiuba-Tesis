// ─────────────────────────────────────────────────────────────────────────────
// DepthProjectionNode  (V1)
//
// Nodo mínimo que demuestra la retroproyección pinhole.
// Suscribe:  imagen de profundidad 16-bit + CameraInfo
// Publica:   PointCloud2 con los puntos 3D resultantes
//
// No aplica ningún filtro, rotación ni grilla.
// Sirve para validar visualmente que la proyección es correcta.
// ─────────────────────────────────────────────────────────────────────────────

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "local_mapper/depth_projector.hpp"

using std::placeholders::_1;

class DepthProjectionNode : public rclcpp::Node {
 public:
  DepthProjectionNode() : rclcpp::Node("depth_projection_node") {
    // ── Parámetros ───────────────────────────────────────────────────────────
    range_min_m_ = static_cast<float>(
        declare_parameter<double>("range_min_m", 0.1));
    range_max_m_ = static_cast<float>(
        declare_parameter<double>("range_max_m", 5.0));

    // ── Publicador ───────────────────────────────────────────────────────────
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/local_mapper/debug/depth_cloud", 10);

    // ── Suscriptores ─────────────────────────────────────────────────────────
    // El nodo usa nombres genéricos (depth/image, depth/camera_info).
    // El launch file remapea desde los topics reales del hardware.
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "depth/image", rclcpp::SensorDataQoS(),
        std::bind(&DepthProjectionNode::onDepth, this, _1));

    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "depth/camera_info", rclcpp::SensorDataQoS(),
        std::bind(&DepthProjectionNode::onCameraInfo, this, _1));

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

  void onDepth(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!projector_) return;

    if (msg->encoding != "16UC1" && msg->encoding != "mono16") {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Encoding inesperado: %s (se espera 16UC1)", msg->encoding.c_str());
      return;
    }

    // Si nadie escucha, no gastar CPU
    if (cloud_pub_->get_subscription_count() == 0) return;

    // Retroproyectar
    const auto* depth_data = reinterpret_cast<const uint16_t*>(msg->data.data());
    auto points = projector_->projectDepthImage(
        depth_data, msg->width, msg->height);

    // Filtrar puntos inválidos y fuera de rango, contar válidos
    std::vector<local_mapper::DepthProjector::Point3D> valid;
    valid.reserve(points.size() / 4);  // reserva conservadora
    for (const auto& pt : points) {
      if (pt.z <= range_min_m_ || pt.z >= range_max_m_) continue;
      valid.push_back(pt);
    }

    publishCloud(msg->header, valid);

    ++frame_count_;
    if (frame_count_ % 30 == 0) {
      RCLCPP_INFO(get_logger(),
          "frame=%zu  puntos_válidos=%zu / %zu  (%.1f%%)",
          frame_count_, valid.size(), points.size(),
          100.0f * valid.size() / points.size());
    }
  }

  void publishCloud(
      const std_msgs::msg::Header& header,
      const std::vector<local_mapper::DepthProjector::Point3D>& pts) {
    auto pc = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pc->header = header;  // mismo frame y stamp que la imagen de profundidad
    pc->height = 1;
    pc->width = static_cast<uint32_t>(pts.size());
    pc->is_dense = true;
    pc->is_bigendian = false;
    pc->point_step = 12;  // 3 floats × 4 bytes
    pc->row_step = pc->point_step * pc->width;

    // Campos XYZ
    auto add_field = [&](const std::string& name, uint32_t offset) {
      sensor_msgs::msg::PointField f;
      f.name = name;
      f.offset = offset;
      f.datatype = sensor_msgs::msg::PointField::FLOAT32;
      f.count = 1;
      pc->fields.push_back(f);
    };
    add_field("x", 0);
    add_field("y", 4);
    add_field("z", 8);

    pc->data.resize(pc->row_step);
    auto* ptr = reinterpret_cast<float*>(pc->data.data());
    for (const auto& pt : pts) {
      *ptr++ = pt.x;
      *ptr++ = pt.y;
      *ptr++ = pt.z;
    }

    cloud_pub_->publish(std::move(pc));
  }

  // ── Miembros ─────────────────────────────────────────────────────────────
  std::unique_ptr<local_mapper::DepthProjector> projector_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

  float range_min_m_ = 0.1f;
  float range_max_m_ = 5.0f;
  size_t frame_count_ = 0;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthProjectionNode>());
  rclcpp::shutdown();
  return 0;
}
