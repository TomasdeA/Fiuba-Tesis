#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>

using std::placeholders::_1;

class SensorTopicRemapperNode : public rclcpp::Node {
 public:
  SensorTopicRemapperNode() : rclcpp::Node("sensor_topic_remapper_node") {
    // Parameters (camera topic remapping)
    depth_image_src_ = declare_parameter<std::string>(
        "depth_image_src", "/camera/camera/depth/image_rect_raw");
    depth_camera_info_src_ = declare_parameter<std::string>(
        "depth_camera_info_src", "/camera/camera/depth/camera_info");
    accel_src_ = declare_parameter<std::string>(
        "accel_src", "/camera/camera/accel/sample");
    gyro_src_ = declare_parameter<std::string>(
        "gyro_src", "/camera/camera/gyro/sample");

    // Subscriptions (source topics)
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        depth_image_src_, rclcpp::SensorDataQoS(),
        std::bind(&SensorTopicRemapperNode::onDepthImage, this, _1));

    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        depth_camera_info_src_, rclcpp::SensorDataQoS(),
        std::bind(&SensorTopicRemapperNode::onCameraInfo, this, _1));

    accel_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        accel_src_, rclcpp::SensorDataQoS(),
        std::bind(&SensorTopicRemapperNode::onAccel, this, _1));

    gyro_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        gyro_src_, rclcpp::SensorDataQoS(),
        std::bind(&SensorTopicRemapperNode::onGyro, this, _1));

    // Publishers (destination topics - normalized)
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>(
        "/sensors/depth/image", rclcpp::SensorDataQoS());

    camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        "/sensors/depth/camera_info", rclcpp::SensorDataQoS());

    accel_pub_ = create_publisher<sensor_msgs::msg::Imu>(
        "/sensors/imu/accel", rclcpp::SensorDataQoS());

    gyro_pub_ = create_publisher<sensor_msgs::msg::Imu>(
        "/sensors/imu/gyro", rclcpp::SensorDataQoS());

    RCLCPP_INFO(get_logger(),
        "SensorTopicRemapperNode initialized.\n"
        "  Depth: %s -> /sensors/depth/image\n"
        "  Info:  %s -> /sensors/depth/camera_info\n"
        "  Accel: %s -> /sensors/imu/accel\n"
        "  Gyro:  %s -> /sensors/imu/gyro",
        depth_image_src_.c_str(), depth_camera_info_src_.c_str(),
        accel_src_.c_str(), gyro_src_.c_str());
  }

 private:
  void onDepthImage(const sensor_msgs::msg::Image::SharedPtr msg) {
    depth_pub_->publish(*msg);
    depth_count_++;
  }

  void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    camera_info_pub_->publish(*msg);
    camera_info_count_++;
  }

  void onAccel(const sensor_msgs::msg::Imu::SharedPtr msg) {
    accel_pub_->publish(*msg);
    accel_count_++;
  }

  void onGyro(const sensor_msgs::msg::Imu::SharedPtr msg) {
    gyro_pub_->publish(*msg);
    gyro_count_++;
  }

  // Subscriptions (source)
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr accel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr gyro_sub_;

  // Publishers (destination)
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr accel_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr gyro_pub_;

  // Parameters
  std::string depth_image_src_;
  std::string depth_camera_info_src_;
  std::string accel_src_;
  std::string gyro_src_;

  // Counters
  size_t depth_count_ = 0;
  size_t camera_info_count_ = 0;
  size_t accel_count_ = 0;
  size_t gyro_count_ = 0;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorTopicRemapperNode>());
  rclcpp::shutdown();
  return 0;
}
