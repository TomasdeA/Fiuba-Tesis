#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <cstdlib>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include "depth_grid_encoder/depth_utils.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class DepthToMatrix : public rclcpp::Node
{
public:
    DepthToMatrix() : rclcpp::Node("depth_to_matrix_node")
    {
        depth_topic_ = this->declare_parameter<std::string>(
            "depth_topic", "/camera/camera/depth/image_rect_raw");

        grid_cfg_.rows = static_cast<int>(this->declare_parameter<int>("rows", 5));
        grid_cfg_.cols = static_cast<int>(this->declare_parameter<int>("cols", 10));
        // if rows = 1 and cols = 2, the output matrix will just indicate a left or right obstacle
        grid_cfg_.z_min_m = static_cast<float>(this->declare_parameter<double>("z_min_m", 0.25));
        grid_cfg_.z_max_m = static_cast<float>(this->declare_parameter<double>("z_max_m", 4.0));

        // Subscriber of depth image (milimeters)
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            depth_topic_, rclcpp::SensorDataQoS(),
            std::bind(&DepthToMatrix::onDepthImage, this, _1));

        // Publisher of min depth grid (meters)
        grid_pub_ = this->create_publisher<custom_interfaces::msg::DepthGrid>(
            "/depth_grid", 10);

        heartbeat_ = this->create_wall_timer(
            1000ms, [this]()
            { RCLCPP_INFO(this->get_logger(),
                          "alive | subcribed to %s | images received: %zu",
                          depth_topic_.c_str(), image_count_); });

        // Watchdog: si no llegan frames en N segundos, matar realsense para forzar reconexion
        watchdog_timeout_s_ = this->declare_parameter<int>("watchdog_timeout_s", 1);
        watchdog_ = this->create_wall_timer(
            1000ms, [this]()
            {
            if (image_count_ == watchdog_last_count_ && image_count_ > 0)
            {
                watchdog_stale_s_++;
                if (watchdog_stale_s_ >= watchdog_timeout_s_ && !watchdog_fired_)
                {
                    RCLCPP_WARN(get_logger(),
                                "[watchdog] Sin frames nuevos por %ds. Reiniciando realsense2_camera_node...",
                                watchdog_stale_s_);
                    std::system("pkill -f realsense2_camera_node");
                    watchdog_fired_ = true;
                    watchdog_stale_s_ = 0;
                }
            }
            else
            {
                watchdog_stale_s_ = 0;
                watchdog_fired_ = false;
            }
            watchdog_last_count_ = image_count_; });

        RCLCPP_INFO(get_logger(),
                    "DepthToMatrix started. Waiting for images on %s",
                    depth_topic_.c_str());
    }

private:
    void onDepthImage(sensor_msgs::msg::Image::SharedPtr msg)
    {
        image_count_++;
        watchdog_stale_s_ = 0; // reset watchdog on every frame

        custom_interfaces::msg::DepthGrid grid; // Output

        const bool ok = tesis_nav::compute_depth_stats(*msg, grid_cfg_, grid);
        if (!ok)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Depth grid computation failed (encoding=%s)", msg->encoding.c_str());
            return;
        }

        grid_pub_->publish(grid);

        // Run with --ros-args --log-level depth_to_matrix_node:=debug
        RCLCPP_DEBUG_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "img %ux%u enc=%s stamp=%u.%u",
            msg->width, msg->height, msg->encoding.c_str(),
            msg->header.stamp.sec, msg->header.stamp.nanosec);
    }

    // Parameters
    std::string depth_topic_;
    tesis_nav::GridConfig grid_cfg_;

    // Image counter
    size_t image_count_ = 0;

    // Watchdog
    int watchdog_timeout_s_{1};
    int watchdog_stale_s_{0};
    size_t watchdog_last_count_{0};
    bool watchdog_fired_{false};

    // Subscriptors
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

    // Publisher
    rclcpp::Publisher<custom_interfaces::msg::DepthGrid>::SharedPtr grid_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr heartbeat_;
    rclcpp::TimerBase::SharedPtr watchdog_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DepthToMatrix>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
