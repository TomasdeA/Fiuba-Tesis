#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

class DepthToMatrix : public rclcpp::Node
{
public:
    DepthToMatrix() : rclcpp::Node("depth_to_matrix_node")
    {
        depth_topic_ = this->declare_parameter<std::string>(
            "depth_topic", "/camera/camera/depth/image_rect_raw");
            
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            depth_topic_, rclcpp::SensorDataQoS(),
            std::bind(&DepthToMatrix::onDepth, this, _1));

        heartbeat_ = this->create_wall_timer(
            1000ms, [this](){
                RCLCPP_INFO(this->get_logger(),
                "alive | sucribed to %s | images received: %zu",
                depth_topic_.c_str(), image_count_);
            }
        );

        RCLCPP_INFO(get_logger(), 
            "DepthToMatrix started. Waiting for images on %s",
            depth_topic_.c_str());
    }

private:
    void onDepth(sensor_msgs::msg::Image::SharedPtr msg){
        image_count_++;

        // Run with --ros-args --log-level depth_to_matrix_node:=debug
        RCLCPP_DEBUG_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "img %ux%u enc=%s stamp=%u.%u",
            msg->width,msg->height,msg->encoding.c_str(),
            msg->header.stamp.sec, msg->header.stamp.nanosec
        );
    }

    // Parameters
    std::string depth_topic_;

    // Image counter
    size_t image_count_ = 0;

    // Subscriptors
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;

    // Timer
    rclcpp::TimerBase::SharedPtr heartbeat_;
};


int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<DepthToMatrix>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}