#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <custom_interfaces/msg/directional_risk.hpp>
#include <custom_interfaces/msg/spatial_awareness.hpp>
#include <limits>
#include <memory>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <string>
#include <utility>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

#include "spatial_awareness/risk_evaluator.hpp"

using namespace std::chrono_literals;

namespace {

struct Vec3 {
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};

  Vec3 operator+(const Vec3& other) const noexcept {
    return {x + other.x, y + other.y, z + other.z};
  }
  Vec3 operator-(const Vec3& other) const noexcept {
    return {x - other.x, y - other.y, z - other.z};
  }
  Vec3 operator*(float scale) const noexcept {
    return {x * scale, y * scale, z * scale};
  }
  float norm() const noexcept { return std::sqrt(x * x + y * y + z * z); }
};

struct Quaternion {
  float w{1.0f};
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};

  Quaternion operator*(const Quaternion& other) const noexcept {
    return {w * other.w - x * other.x - y * other.y - z * other.z,
            w * other.x + x * other.w + y * other.z - z * other.y,
            w * other.y - x * other.z + y * other.w + z * other.x,
            w * other.z + x * other.y - y * other.x + z * other.w};
  }

  Quaternion normalized() const noexcept {
    const float length = std::sqrt(w * w + x * x + y * y + z * z);
    if (length < 1e-8f) {
      return {};
    }
    const float inverse = 1.0f / length;
    return {w * inverse, x * inverse, y * inverse, z * inverse};
  }

  Vec3 rotate(const Vec3& vector) const noexcept {
    const Vec3 qv{x, y, z};
    const Vec3 cross1{qv.y * vector.z - qv.z * vector.y,
                      qv.z * vector.x - qv.x * vector.z,
                      qv.x * vector.y - qv.y * vector.x};
    const Vec3 cross2{qv.y * cross1.z - qv.z * cross1.y,
                      qv.z * cross1.x - qv.x * cross1.z,
                      qv.x * cross1.y - qv.y * cross1.x};
    return vector + cross1 * (2.0f * w) + cross2 * 2.0f;
  }
};

bool finite(float value) { return std::isfinite(value); }

}  // namespace

class SpatialAwarenessNode : public rclcpp::Node {
 public:
  SpatialAwarenessNode() : Node("spatial_awareness") {
    occupancy_grid_topic_ = declare_parameter<std::string>(
        "occupancy_grid_topic", "/local_mapper/occupancy_grid");
    odometry_topic_ =
        declare_parameter<std::string>("odometry_topic", "nav_odom");
    aperture_state_topic_ = declare_parameter<std::string>(
        "aperture_state_topic", "/perception/depth_grid/aperture_state_deg");
    output_topic_ = declare_parameter<std::string>(
        "output_topic", "/spatial_awareness/collision_risk");
    debug_markers_topic_ = declare_parameter<std::string>(
        "debug_markers_topic", "/spatial_awareness/debug/markers");
    publish_debug_markers_ =
        declare_parameter<bool>("publish_debug_markers", true);

    occupancy_threshold_ = declare_parameter<int>("occupancy_threshold", 65);
    velocity_filter_alpha_ = static_cast<float>(
        declare_parameter<double>("velocity_filter_alpha", 0.25));
    max_velocity_mps_ =
        static_cast<float>(declare_parameter<double>("max_velocity_mps", 3.0));
    twist_consistency_tolerance_mps_ = static_cast<float>(
        declare_parameter<double>("twist_consistency_tolerance_mps", 0.35));
    odom_timeout_s_ = declare_parameter<double>("odom_timeout_s", 0.40);
    map_timeout_s_ = declare_parameter<double>("map_timeout_s", 0.40);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);
    intensity_slew_rate_per_s_ = static_cast<float>(
        declare_parameter<double>("intensity_slew_rate_per_s", 200.0));
    aperture_half_angle_deg_ = static_cast<float>(
        declare_parameter<double>("default_aperture_deg", 45.0));

    spatial_awareness::RiskEvaluator::Config evaluator_config;
    evaluator_config.min_cluster_cells =
        declare_parameter<int>("min_cluster_cells", 3);
    evaluator_config.min_distance_m =
        static_cast<float>(declare_parameter<double>("min_distance_m", 0.20));
    evaluator_config.max_distance_m =
        static_cast<float>(declare_parameter<double>("max_distance_m", 1.00));
    evaluator_config.body_radius_m =
        static_cast<float>(declare_parameter<double>("body_radius_m", 0.20));
    min_motion_speed_mps_ = static_cast<float>(
        declare_parameter<double>("min_motion_speed_mps", 0.10));
    evaluator_config.min_motion_speed_mps = min_motion_speed_mps_;
    evaluator_config.min_closing_speed_mps = static_cast<float>(
        declare_parameter<double>("min_closing_speed_mps", 0.10));
    evaluator_config.closing_speed_hysteresis_mps = static_cast<float>(
        declare_parameter<double>("closing_speed_hysteresis_mps", 0.03));
    evaluator_config.ttc_min_s =
        static_cast<float>(declare_parameter<double>("ttc_min_s", 0.25));
    evaluator_config.ttc_max_s =
        static_cast<float>(declare_parameter<double>("ttc_max_s", 2.00));
    evaluator_config.fov_margin_deg =
        static_cast<float>(declare_parameter<double>("fov_margin_deg", 3.0));
    evaluator_config.rear_half_angle_deg = static_cast<float>(
        declare_parameter<double>("rear_half_angle_deg", 45.0));
    evaluator_ =
        std::make_unique<spatial_awareness::RiskEvaluator>(evaluator_config);

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        occupancy_grid_topic_, rclcpp::QoS(1).transient_local(),
        std::bind(&SpatialAwarenessNode::onMap, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odometry_topic_, rclcpp::QoS(5).reliable(),
        std::bind(&SpatialAwarenessNode::onOdom, this, std::placeholders::_1));
    aperture_sub_ = create_subscription<std_msgs::msg::Float32>(
        aperture_state_topic_, rclcpp::QoS(1).transient_local(),
        [this](const std_msgs::msg::Float32::SharedPtr message) {
          if (finite(message->data) && message->data > 0.0f) {
            aperture_half_angle_deg_ = message->data;
          }
        });
    risk_pub_ = create_publisher<custom_interfaces::msg::SpatialAwareness>(
        output_topic_, 10);
    if (publish_debug_markers_) {
      marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
          debug_markers_topic_, 10);
    }

    const double bounded_rate = std::max(1.0, publish_rate_hz_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / bounded_rate)),
        std::bind(&SpatialAwarenessNode::evaluateAndPublish, this));

    last_publish_time_ = now();
    RCLCPP_INFO(get_logger(),
                "SpatialAwareness listo. map=%s odom=%s aperture=%s output=%s",
                occupancy_grid_topic_.c_str(), odometry_topic_.c_str(),
                aperture_state_topic_.c_str(), output_topic_.c_str());
  }

 private:
  static Quaternion messageQuaternion(
      const geometry_msgs::msg::Quaternion& message) {
    return Quaternion{
        static_cast<float>(message.w), static_cast<float>(message.x),
        static_cast<float>(message.y), static_cast<float>(message.z)}
        .normalized();
  }

  static Quaternion rtabMapToInternal(const Quaternion& ros_orientation) {
    static constexpr Quaternion kRosToInternal{0.5f, 0.5f, -0.5f, 0.5f};
    return (kRosToInternal * ros_orientation).normalized();
  }

  static Vec3 rtabMapPositionToInternal(const Vec3& ros_position) {
    return {-ros_position.y, -ros_position.z, ros_position.x};
  }

  void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr message) {
    const std::size_t expected_size =
        static_cast<std::size_t>(message->info.width) *
        static_cast<std::size_t>(message->info.height);
    if (message->data.size() < expected_size ||
        message->info.resolution <= 0.0f) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "OccupancyGrid invalido: data=%zu expected=%zu resolution=%.3f",
          message->data.size(), expected_size, message->info.resolution);
      return;
    }

    const Quaternion origin_orientation =
        messageQuaternion(message->info.origin.orientation);
    const Vec3 origin{static_cast<float>(message->info.origin.position.x),
                      static_cast<float>(message->info.origin.position.y),
                      static_cast<float>(message->info.origin.position.z)};
    const float resolution = message->info.resolution;

    std::vector<spatial_awareness::OccupiedCell> decoded;
    decoded.reserve(expected_size / 20);
    for (std::uint32_t row = 0; row < message->info.height; ++row) {
      for (std::uint32_t column = 0; column < message->info.width; ++column) {
        const std::size_t index =
            static_cast<std::size_t>(column) +
            static_cast<std::size_t>(row) * message->info.width;
        if (message->data[index] < occupancy_threshold_) {
          continue;
        }

        const Vec3 local{(static_cast<float>(column) + 0.5f) * resolution,
                         (static_cast<float>(row) + 0.5f) * resolution, 0.0f};
        const Vec3 world = origin + origin_orientation.rotate(local);
        decoded.push_back({{world.x, world.z},
                           static_cast<int>(column),
                           static_cast<int>(row)});
      }
    }

    occupied_cells_ = std::move(decoded);
    cell_size_m_ = resolution;
    last_map_received_ = now();
    has_map_ = true;
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr message) {
    const rclcpp::Time stamp(message->header.stamp);
    const Vec3 ros_position{static_cast<float>(message->pose.pose.position.x),
                            static_cast<float>(message->pose.pose.position.y),
                            static_cast<float>(message->pose.pose.position.z)};
    const Vec3 internal_position = rtabMapPositionToInternal(ros_position);
    const Quaternion internal_orientation =
        rtabMapToInternal(messageQuaternion(message->pose.pose.orientation));
    const Vec3 forward3 = internal_orientation.rotate({0.0f, 0.0f, 1.0f});
    pose_.position = {internal_position.x, internal_position.z};
    pose_.forward =
        nav_math::Vec2{forward3.x, forward3.z}.normalized();

    nav_math::Vec2 pose_velocity;
    bool pose_velocity_valid = false;
    if (has_previous_pose_) {
      const double dt = (stamp - previous_odom_stamp_).seconds();
      if (dt > 0.01 && dt < 1.0) {
        pose_velocity = {
            static_cast<float>((internal_position.x - previous_position_.x) /
                               dt),
            static_cast<float>((internal_position.z - previous_position_.z) /
                               dt)};
        pose_velocity_valid = finite(pose_velocity.x) &&
                              finite(pose_velocity.z) &&
                              pose_velocity.norm() <= max_velocity_mps_;
      }
    }

    // nav_msgs/Odometry defines twist in child_frame_id. RTAB-Map is launched
    // with camera_color_optical_frame, so the vector is rotated into odom.
    const Vec3 twist_body{static_cast<float>(message->twist.twist.linear.x),
                          static_cast<float>(message->twist.twist.linear.y),
                          static_cast<float>(message->twist.twist.linear.z)};
    const Vec3 twist_world = internal_orientation.rotate(twist_body);
    const nav_math::Vec2 twist_velocity{twist_world.x, twist_world.z};
    const bool twist_valid = finite(twist_velocity.x) &&
                             finite(twist_velocity.z) &&
                             twist_velocity.norm() <= max_velocity_mps_;

    nav_math::Vec2 selected_velocity;
    bool selected_valid = false;
    if (twist_valid && pose_velocity_valid) {
      const float disagreement = (twist_velocity - pose_velocity).norm();
      const bool twist_misses_detected_motion =
          pose_velocity.norm() >= min_motion_speed_mps_ &&
          twist_velocity.norm() < 0.5f * min_motion_speed_mps_;
      if (!twist_misses_detected_motion &&
          disagreement <= twist_consistency_tolerance_mps_) {
        selected_velocity = twist_velocity;
      } else {
        selected_velocity = pose_velocity;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                             "twist RTAB-Map inconsistente con pose "
                             "(error=%.2fm/s); usando dpose/dt",
                             disagreement);
      }
      selected_valid = true;
    } else if (twist_valid) {
      selected_velocity = twist_velocity;
      selected_valid = true;
    } else if (pose_velocity_valid) {
      selected_velocity = pose_velocity;
      selected_valid = true;
    }

    if (selected_valid) {
      const float alpha = std::clamp(velocity_filter_alpha_, 0.0f, 1.0f);
      if (!has_filtered_velocity_) {
        filtered_velocity_ = selected_velocity;
        has_filtered_velocity_ = true;
      } else {
        filtered_velocity_ =
            selected_velocity * alpha + filtered_velocity_ * (1.0f - alpha);
      }
    } else {
      filtered_velocity_ = {};
      has_filtered_velocity_ = false;
    }

    previous_position_ = internal_position;
    previous_odom_stamp_ = stamp;
    has_previous_pose_ = true;
    last_odom_received_ = now();
    has_odom_ = true;
  }

  static void fillRiskMessage(
      const spatial_awareness::DirectionalRisk& risk,
      custom_interfaces::msg::DirectionalRisk& message) {
    message.active = risk.active;
    message.intensity = risk.intensity;
    message.distance_m = risk.distance_m;
    message.closing_speed_mps = risk.closing_speed_mps;
    message.time_to_collision_s = risk.time_to_collision_s;
  }

  void applySlew(spatial_awareness::DirectionalRisk& risk,
                 float& previous_intensity, float max_delta) {
    if (!risk.active) {
      previous_intensity = 0.0f;
      risk.intensity = 0.0f;
      return;
    }
    risk.intensity = std::clamp(
        risk.intensity, std::max(0.0f, previous_intensity - max_delta),
        std::min(100.0f, previous_intensity + max_delta));
    previous_intensity = risk.intensity;
  }

  void evaluateAndPublish() {
    const rclcpp::Time current_time = now();
    custom_interfaces::msg::SpatialAwareness output;
    output.header.stamp = current_time;
    output.header.frame_id = "odom";

    const bool map_fresh =
        has_map_ &&
        (current_time - last_map_received_).seconds() <= map_timeout_s_;
    const bool odom_fresh =
        has_odom_ &&
        (current_time - last_odom_received_).seconds() <= odom_timeout_s_;
    output.valid = map_fresh && odom_fresh && has_filtered_velocity_;

    const double elapsed =
        std::max(0.0, (current_time - last_publish_time_).seconds());
    const float max_intensity_delta =
        intensity_slew_rate_per_s_ * static_cast<float>(elapsed);
    last_publish_time_ = current_time;

    spatial_awareness::RiskResult result;
    if (output.valid) {
      // Grid resolution can change at runtime, so rebuild the evaluator only
      // when the map reports a different cell size.
      if (std::abs(cell_size_m_ - evaluator_cell_size_m_) > 1e-5f) {
        rebuildEvaluatorForCellSize();
      }
      result = evaluator_->evaluate(pose_, filtered_velocity_,
                                    aperture_half_angle_deg_, occupied_cells_,
                                    previous_active_directions_);
      applySlew(result.left, previous_left_intensity_, max_intensity_delta);
      applySlew(result.right, previous_right_intensity_, max_intensity_delta);
      applySlew(result.rear, previous_rear_intensity_, max_intensity_delta);
      previous_active_directions_ = {result.left.active, result.right.active,
                                     result.rear.active};
    } else {
      previous_left_intensity_ = 0.0f;
      previous_right_intensity_ = 0.0f;
      previous_rear_intensity_ = 0.0f;
      previous_active_directions_ = {};
    }

    fillRiskMessage(result.left, output.left);
    fillRiskMessage(result.right, output.right);
    fillRiskMessage(result.rear, output.rear);
    risk_pub_->publish(output);
    publishMarkers(current_time, output.valid, result);
  }

  geometry_msgs::msg::Point markerPoint(float x, float z) const {
    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = 0.0;
    point.z = z;
    return point;
  }

  void publishMarkers(const rclcpp::Time& stamp, bool valid,
                      const spatial_awareness::RiskResult& result) {
    if (!marker_pub_ || marker_pub_->get_subscription_count() == 0) {
      return;
    }

    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    if (valid) {
      const float aperture_rad =
          aperture_half_angle_deg_ * 3.14159265358979323846f / 180.0f;
      const auto forward = pose_.forward.normalized();
      const auto rotate = [](const nav_math::Vec2& vector,
                             float angle) {
        return nav_math::Vec2{
            vector.x * std::cos(angle) + vector.z * std::sin(angle),
            -vector.x * std::sin(angle) + vector.z * std::cos(angle)};
      };

      visualization_msgs::msg::Marker fov;
      fov.header.stamp = stamp;
      fov.header.frame_id = "odom";
      fov.ns = "fov";
      fov.id = 0;
      fov.type = visualization_msgs::msg::Marker::LINE_LIST;
      fov.action = visualization_msgs::msg::Marker::ADD;
      fov.scale.x = 0.025;
      fov.color.r = 0.2f;
      fov.color.g = 0.7f;
      fov.color.b = 1.0f;
      fov.color.a = 0.9f;
      for (const float sign : {-1.0f, 1.0f}) {
        const auto edge = rotate(forward, sign * aperture_rad);
        fov.points.push_back(markerPoint(pose_.position.x, pose_.position.z));
        fov.points.push_back(
            markerPoint(pose_.position.x + edge.x, pose_.position.z + edge.z));
      }
      markers.markers.push_back(fov);

      visualization_msgs::msg::Marker velocity;
      velocity.header = fov.header;
      velocity.ns = "velocity";
      velocity.id = 0;
      velocity.type = visualization_msgs::msg::Marker::ARROW;
      velocity.action = visualization_msgs::msg::Marker::ADD;
      velocity.scale.x = 0.04;
      velocity.scale.y = 0.08;
      velocity.scale.z = 0.08;
      velocity.color.r = 1.0f;
      velocity.color.g = 0.8f;
      velocity.color.a = 0.9f;
      velocity.points.push_back(
          markerPoint(pose_.position.x, pose_.position.z));
      velocity.points.push_back(
          markerPoint(pose_.position.x + filtered_velocity_.x,
                      pose_.position.z + filtered_velocity_.z));
      markers.markers.push_back(velocity);

      const auto addRiskMarker =
          [this, &markers, &fov](const spatial_awareness::DirectionalRisk& risk,
                                 int id, float red, float green, float blue) {
            if (!risk.active) {
              return;
            }
            visualization_msgs::msg::Marker marker;
            marker.header = fov.header;
            marker.ns = "selected_risk";
            marker.id = id;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position =
                markerPoint(risk.obstacle_position.x, risk.obstacle_position.z);
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.16;
            marker.scale.y = 0.16;
            marker.scale.z = 0.16;
            marker.color.r = red;
            marker.color.g = green;
            marker.color.b = blue;
            marker.color.a = std::max(0.25f, risk.intensity / 100.0f);
            markers.markers.push_back(marker);
          };
      addRiskMarker(result.left, 0, 1.0f, 0.2f, 0.8f);
      addRiskMarker(result.right, 1, 0.2f, 0.8f, 1.0f);
      addRiskMarker(result.rear, 2, 1.0f, 0.3f, 0.1f);
    }

    marker_pub_->publish(markers);
  }

  void rebuildEvaluatorForCellSize() {
    spatial_awareness::RiskEvaluator::Config config;
    config.min_cluster_cells = get_parameter("min_cluster_cells").as_int();
    config.min_distance_m =
        static_cast<float>(get_parameter("min_distance_m").as_double());
    config.max_distance_m =
        static_cast<float>(get_parameter("max_distance_m").as_double());
    config.body_radius_m =
        static_cast<float>(get_parameter("body_radius_m").as_double());
    config.cell_size_m = cell_size_m_;
    config.min_motion_speed_mps =
        static_cast<float>(get_parameter("min_motion_speed_mps").as_double());
    config.min_closing_speed_mps =
        static_cast<float>(get_parameter("min_closing_speed_mps").as_double());
    config.closing_speed_hysteresis_mps = static_cast<float>(
        get_parameter("closing_speed_hysteresis_mps").as_double());
    config.ttc_min_s =
        static_cast<float>(get_parameter("ttc_min_s").as_double());
    config.ttc_max_s =
        static_cast<float>(get_parameter("ttc_max_s").as_double());
    config.fov_margin_deg =
        static_cast<float>(get_parameter("fov_margin_deg").as_double());
    config.rear_half_angle_deg =
        static_cast<float>(get_parameter("rear_half_angle_deg").as_double());
    evaluator_ = std::make_unique<spatial_awareness::RiskEvaluator>(config);
    evaluator_cell_size_m_ = cell_size_m_;
  }

  std::string occupancy_grid_topic_;
  std::string odometry_topic_;
  std::string aperture_state_topic_;
  std::string output_topic_;
  std::string debug_markers_topic_;
  bool publish_debug_markers_{true};

  int occupancy_threshold_{65};
  float velocity_filter_alpha_{0.25f};
  float max_velocity_mps_{3.0f};
  float twist_consistency_tolerance_mps_{0.35f};
  float min_motion_speed_mps_{0.10f};
  double odom_timeout_s_{0.40};
  double map_timeout_s_{0.40};
  double publish_rate_hz_{10.0};
  float intensity_slew_rate_per_s_{200.0f};
  float aperture_half_angle_deg_{45.0f};

  std::unique_ptr<spatial_awareness::RiskEvaluator> evaluator_;
  float evaluator_cell_size_m_{0.10f};
  float cell_size_m_{0.10f};
  std::vector<spatial_awareness::OccupiedCell> occupied_cells_;
  spatial_awareness::Pose2D pose_;
  nav_math::Vec2 filtered_velocity_;
  spatial_awareness::ActiveDirections previous_active_directions_;
  Vec3 previous_position_;
  rclcpp::Time previous_odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_map_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};
  bool has_map_{false};
  bool has_odom_{false};
  bool has_previous_pose_{false};
  bool has_filtered_velocity_{false};
  float previous_left_intensity_{0.0f};
  float previous_right_intensity_{0.0f};
  float previous_rear_intensity_{0.0f};

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr aperture_sub_;
  rclcpp::Publisher<custom_interfaces::msg::SpatialAwareness>::SharedPtr
      risk_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpatialAwarenessNode>());
  rclcpp::shutdown();
  return 0;
}
