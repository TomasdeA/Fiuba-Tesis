#include <rclcpp/rclcpp.hpp>
#include <chrono>
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
            "depth_topic", "/sensors/depth/image");

        grid_cfg_.rows  = static_cast<int>(this->declare_parameter<int>("rows", 5));
        grid_cfg_.cols  = static_cast<int>(this->declare_parameter<int>("cols", 10));
        // Si filas = 1 y columnas = 2, la matriz de salida solo indicará un obstáculo a izquierda o derecha
        grid_cfg_.z_min_m = static_cast<float>(this->declare_parameter<double>("z_min_m", 0.25));
        grid_cfg_.z_max_m = static_cast<float>(this->declare_parameter<double>("z_max_m", 4.0));


        // Suscriptor de imagen de profundidad (milímetros)
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            depth_topic_, rclcpp::SensorDataQoS(),
            std::bind(&DepthToMatrix::onDepthImage, this, _1));

        // Publicador de grilla de profundidad mínima (metros)
        grid_pub_ = this->create_publisher<custom_interfaces::msg::DepthGrid>(
            "/perception/depth_grid", 10);

        heartbeat_ = this->create_wall_timer(
            1000ms, [this](){
                RCLCPP_DEBUG(this->get_logger(),
                "activo | suscrito a %s | imágenes recibidas: %zu",
                depth_topic_.c_str(), image_count_);
            }
        );

        RCLCPP_INFO(get_logger(),
            "DepthToMatrix iniciado. Esperando imágenes desde %s",
            depth_topic_.c_str());
    }

private:
    void onDepthImage(sensor_msgs::msg::Image::SharedPtr msg){
        image_count_++;

        custom_interfaces::msg::DepthGrid grid; // Salida

        const bool ok = tesis_nav::compute_depth_stats(*msg, grid_cfg_, grid);
        if (!ok) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Fallo en cálculo de grilla de profundidad (encoding=%s)", msg->encoding.c_str());
            return;
        }

        grid_pub_->publish(grid);

        // Ejecutar con --ros-args --log-level depth_to_matrix_node:=debug
        RCLCPP_DEBUG_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "img %ux%u enc=%s stamp=%u.%u",
            msg->width,msg->height,msg->encoding.c_str(),
            msg->header.stamp.sec, msg->header.stamp.nanosec
        );

    }

    std::string depth_topic_;
    tesis_nav::GridConfig grid_cfg_;

    size_t image_count_ = 0;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

    rclcpp::Publisher<custom_interfaces::msg::DepthGrid>::SharedPtr grid_pub_;

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
