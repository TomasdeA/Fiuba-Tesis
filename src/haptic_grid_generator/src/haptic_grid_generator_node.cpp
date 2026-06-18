#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <custom_interfaces/msg/depth_cell_stats.hpp>
#include <custom_interfaces/msg/depth_grid.hpp>
#include <custom_interfaces/msg/directional_risk.hpp>
#include <custom_interfaces/msg/haptic_grid.hpp>
#include <custom_interfaces/msg/pipeline_mode.hpp>
#include <custom_interfaces/msg/spatial_awareness.hpp>
#include <rclcpp/rclcpp.hpp>

namespace {

float clamp(float value, float low, float high)
{
  return std::max(low, std::min(value, high));
}

bool hasValidDistance(const custom_interfaces::msg::DepthCellStats& cell)
{
  return cell.count > 0 && std::isfinite(cell.min_m) && cell.min_m > 0.0f;
}

}  // namespace

class HapticGridGeneratorNode : public rclcpp::Node
{
public:
  HapticGridGeneratorNode() : rclcpp::Node("haptic_grid_generator")
  {
    input_grid_topic_ = declare_parameter<std::string>(
        "input_grid_topic", "/perception/depth_grid");
    spatial_awareness_topic_ = declare_parameter<std::string>(
        "spatial_awareness_topic", "/spatial_awareness/collision_risk");
    output_grid_topic_ = declare_parameter<std::string>(
        "output_grid_topic", "/perception/haptic_grid");
    pipeline_mode_topic_ = declare_parameter<std::string>(
      "pipeline_mode_topic", "/pipeline/selected_mode");

    z_min_m_ = static_cast<float>(declare_parameter<double>("z_min_m", 0.25));
    z_max_m_ = static_cast<float>(declare_parameter<double>("z_max_m", 3.0));
    spatial_timeout_s_ =
        declare_parameter<double>("spatial_timeout_s", 0.75);
    left_column_count_ = std::max(
        0, static_cast<int>(declare_parameter<int>("left_column_count", 1)));
    right_column_count_ = std::max(
        0, static_cast<int>(declare_parameter<int>("right_column_count", 1)));
    rear_column_count_ = std::max(
        0, static_cast<int>(declare_parameter<int>("rear_column_count", 2)));

    if (z_max_m_ <= z_min_m_) {
      RCLCPP_WARN(get_logger(),
                  "z_max_m <= z_min_m; ajustando z_max_m a z_min_m + 1.0");
      z_max_m_ = z_min_m_ + 1.0f;
    }

    grid_pub_ = create_publisher<custom_interfaces::msg::HapticGrid>(
        output_grid_topic_, 10);
    grid_sub_ = create_subscription<custom_interfaces::msg::DepthGrid>(
        input_grid_topic_, 10,
        std::bind(&HapticGridGeneratorNode::onGrid, this,
                  std::placeholders::_1));
    spatial_sub_ =
        create_subscription<custom_interfaces::msg::SpatialAwareness>(
            spatial_awareness_topic_, 10,
            std::bind(&HapticGridGeneratorNode::onSpatialAwareness, this,
                      std::placeholders::_1));
    pipeline_mode_sub_ =
      create_subscription<custom_interfaces::msg::PipelineMode>(
        pipeline_mode_topic_, 10,
        std::bind(&HapticGridGeneratorNode::onPipelineMode, this,
              std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
          "HapticGridGenerator listo. input=%s spatial=%s output=%s mode=%s",
          input_grid_topic_.c_str(), spatial_awareness_topic_.c_str(),
          output_grid_topic_.c_str(), pipeline_mode_topic_.c_str());
  }

private:
  void onSpatialAwareness(
      const custom_interfaces::msg::SpatialAwareness::SharedPtr msg)
  {
    latest_spatial_ = *msg;
    latest_spatial_stamp_ = now();
    has_spatial_ = true;
  }

  void onPipelineMode(
      const custom_interfaces::msg::PipelineMode::SharedPtr msg)
  {
    pipeline_mode_ = static_cast<int>(msg->mode);
  }

  void onGrid(const custom_interfaces::msg::DepthGrid::SharedPtr msg)
  {
    const int rows = msg->rows;
    const int cols = msg->cols;
    const int expected = rows * cols;
    if (rows <= 0 || cols <= 0 ||
        static_cast<int>(msg->cells.size()) < expected) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "DepthGrid invalido rows=%d cols=%d cells=%zu",
                           rows, cols, msg->cells.size());
      return;
    }

    custom_interfaces::msg::HapticGrid out;
    out.header = msg->header;
    out.rows = rows;
    out.cols = cols;
    out.intensities.assign(static_cast<std::size_t>(expected), 0.0f);
    out.active.assign(static_cast<std::size_t>(expected), false);

    for (int i = 0; i < expected; ++i) {
      const auto& cell = msg->cells[static_cast<std::size_t>(i)];
      if (!hasValidDistance(cell)) {
        continue;
      }
      out.intensities[static_cast<std::size_t>(i)] =
          distanceToIntensity(cell.min_m);
      out.active[static_cast<std::size_t>(i)] = true;
    }

    if (spatialAwarenessIsUsable()) {
      overlayLeft(out);
      overlayRight(out);
      overlayRear(out);
    }

    grid_pub_->publish(std::move(out));
  }

  bool spatialAwarenessIsUsable() const
  {
    if (pipeline_mode_ != static_cast<int>(custom_interfaces::msg::PipelineMode::MODE_FULL)) {
      return false;
    }
    if (!has_spatial_ || !latest_spatial_.valid) {
      return false;
    }
    return (now() - latest_spatial_stamp_).seconds() <=
           std::max(0.05, spatial_timeout_s_);
  }

  float distanceToIntensity(float distance_m) const
  {
    if (!std::isfinite(distance_m) || distance_m <= 0.0f) {
      return 0.0f;
    }
    const float distance = clamp(distance_m, z_min_m_, z_max_m_);
    return (z_max_m_ - distance) / (z_max_m_ - z_min_m_) * 100.0f;
  }

  void applyRiskToColumn(custom_interfaces::msg::HapticGrid& grid, int col,
                         float intensity)
  {
    if (col < 0 || col >= grid.cols || intensity <= 0.0f) {
      return;
    }

    const float risk_intensity = clamp(intensity, 0.0f, 100.0f);
    for (int row = 0; row < grid.rows; ++row) {
      const auto idx = static_cast<std::size_t>(row * grid.cols + col);
      grid.intensities[idx] = std::max(grid.intensities[idx], risk_intensity);
      grid.active[idx] = true;
    }
  }

  void overlayLeft(custom_interfaces::msg::HapticGrid& grid)
  {
    const auto& risk = latest_spatial_.left;
    if (!risk.active) {
      return;
    }
    const int count = std::min(left_column_count_, grid.cols);
    for (int col = 0; col < count; ++col) {
      applyRiskToColumn(grid, col, risk.intensity);
    }
  }

  void overlayRight(custom_interfaces::msg::HapticGrid& grid)
  {
    const auto& risk = latest_spatial_.right;
    if (!risk.active) {
      return;
    }
    const int count = std::min(right_column_count_, grid.cols);
    for (int i = 0; i < count; ++i) {
      applyRiskToColumn(grid, grid.cols - 1 - i, risk.intensity);
    }
  }

  void overlayRear(custom_interfaces::msg::HapticGrid& grid)
  {
    const auto& risk = latest_spatial_.rear;
    if (!risk.active || rear_column_count_ <= 0) {
      return;
    }

    const int count = std::min(rear_column_count_, grid.cols);
    const int start = std::max(0, (grid.cols - count) / 2);
    for (int i = 0; i < count; ++i) {
      applyRiskToColumn(grid, start + i, risk.intensity);
    }
  }

  std::string input_grid_topic_;
  std::string spatial_awareness_topic_;
  std::string output_grid_topic_;
  std::string pipeline_mode_topic_;

  float z_min_m_{0.25f};
  float z_max_m_{3.0f};
  double spatial_timeout_s_{0.75};
  int left_column_count_{1};
  int right_column_count_{1};
  int rear_column_count_{2};

  bool has_spatial_{false};
  int pipeline_mode_{static_cast<int>(custom_interfaces::msg::PipelineMode::MODE_RAW)};
  custom_interfaces::msg::SpatialAwareness latest_spatial_;
  rclcpp::Time latest_spatial_stamp_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<custom_interfaces::msg::DepthGrid>::SharedPtr grid_sub_;
  rclcpp::Subscription<custom_interfaces::msg::SpatialAwareness>::SharedPtr
      spatial_sub_;
  rclcpp::Subscription<custom_interfaces::msg::PipelineMode>::SharedPtr
      pipeline_mode_sub_;
  rclcpp::Publisher<custom_interfaces::msg::HapticGrid>::SharedPtr grid_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HapticGridGeneratorNode>());
  rclcpp::shutdown();
  return 0;
}
