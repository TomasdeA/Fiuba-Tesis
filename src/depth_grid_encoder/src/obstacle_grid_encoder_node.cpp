#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <chrono>
#include <cmath>
#include <limits>

#include "depth_grid_encoder/depth_utils.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

// Proyecta una nube de puntos de obstáculos (PointCloud2 en gravity_aligned_frame)
// sobre una grilla 2D XY de rows×cols celdas.
// Cada celda acumula estadísticas de la coordenada Z (profundidad hacia adelante)
// de todos los puntos cuyas coordenadas X,Y caen dentro de la celda.
// En particular, min_m captura el obstáculo más cercano en cada celda.
//
// Eje Y: origen en la cámara (montada en la cabeza del usuario).
//   y_min = -0.20 m  (guardia de seguridad — el obstacle_cloud ya tiene techo removido)
//   y_max =  2.20 m  (guardia de seguridad — el obstacle_cloud ya tiene suelo removido)
// El obstacle_cloud ya tiene el suelo y el techo removidos por local_mapper;
// estos límites actúan únicamente como guardia ante puntos espurios.
class ObstacleGridEncoder : public rclcpp::Node
{
public:
    ObstacleGridEncoder() : rclcpp::Node("obstacle_grid_encoder")
    {
        cloud_topic_ = this->declare_parameter<std::string>(
            "cloud_topic", "/local_mapper/obstacle_cloud");

        cfg_.rows    = static_cast<int>(this->declare_parameter<int>("rows", 5));
        cfg_.cols    = static_cast<int>(this->declare_parameter<int>("cols", 10));
        cfg_.z_min_m = static_cast<float>(this->declare_parameter<double>("z_min_m", 0.25));
        cfg_.z_max_m = static_cast<float>(this->declare_parameter<double>("z_max_m", 5.0));

        x_min_ = static_cast<float>(this->declare_parameter<double>("x_min_m", -3.0));
        x_max_ = static_cast<float>(this->declare_parameter<double>("x_max_m",  3.0));

        // Y: constantes fijas de seguridad. El obstacle_cloud ya viene sin suelo ni techo
        // (removidos por local_mapper); estos límites solo descartan puntos espurios.
        y_min_ = -0.20f;   // 20 cm por encima de la cámara
        y_max_ =  2.20f;   // 2.2 m por debajo de la cámara

        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic_, rclcpp::SensorDataQoS(),
            std::bind(&ObstacleGridEncoder::onCloud, this, _1));

        grid_pub_ = this->create_publisher<custom_interfaces::msg::DepthGrid>(
            "/perception/depth_grid", 10);

        heartbeat_ = this->create_wall_timer(
            1000ms, [this]() {
                RCLCPP_DEBUG(this->get_logger(),
                    "activo | suscrito a %s | nubes recibidas: %zu",
                    cloud_topic_.c_str(), cloud_count_);
            });

        RCLCPP_INFO(get_logger(),
            "ObstacleGridEncoder iniciado. Grid=%dx%d, X=[%.1f,%.1f], "
            "Y=[%.2f,%.2f] (guardia seguridad), Z=[%.2f,%.2f]. Esperando nube en %s",
            cfg_.rows, cfg_.cols,
            x_min_, x_max_, y_min_, y_max_,
            cfg_.z_min_m, cfg_.z_max_m,
            cloud_topic_.c_str());
    }

private:
    void onCloud(sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        ++cloud_count_;

        const int rows = cfg_.rows;
        const int cols = cfg_.cols;
        const int n    = rows * cols;

        // Acumuladores por celda
        std::vector<float>  z_min(n, std::numeric_limits<float>::infinity());
        std::vector<float>  z_max(n, -std::numeric_limits<float>::infinity());
        std::vector<double> z_sum(n, 0.0);
        std::vector<int>    z_cnt(n, 0);

        const float x_range = x_max_ - x_min_;
        const float y_range = y_max_ - y_min_;

        try {
            sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
            {
                const float px = *iter_x;
                const float py = *iter_y;
                const float pz = *iter_z;

                // Descartar puntos con coordenadas inválidas (NaN/Inf)
                if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) continue;

                // Filtro de profundidad
                if (pz < cfg_.z_min_m || pz > cfg_.z_max_m) continue;

                // Mapear X →  columna
                if (px < x_min_ || px >= x_max_) continue;
                const int c = static_cast<int>((px - x_min_) / x_range * cols);
                if (c < 0 || c >= cols) continue;

                // Mapear Y → fila
                if (py < y_min_ || py >= y_max_) continue;
                const int r = static_cast<int>((py - y_min_) / y_range * rows);
                if (r < 0 || r >= rows) continue;

                const int idx = r * cols + c;
                if (pz < z_min[idx]) z_min[idx] = pz;
                if (pz > z_max[idx]) z_max[idx] = pz;
                z_sum[idx] += pz;
                ++z_cnt[idx];
            }
        } catch (const std::runtime_error & e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Error al iterar PointCloud2: %s", e.what());
            return;
        }

        // Construir mensaje de salida
        custom_interfaces::msg::DepthGrid grid;
        tesis_nav::allocate(grid, static_cast<uint32_t>(rows), static_cast<uint32_t>(cols));
        grid.header = msg->header;

        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                const int idx = r * cols + c;
                auto & cell = tesis_nav::at(grid, static_cast<size_t>(r), static_cast<size_t>(c));
                if (z_cnt[idx] > 0) {
                    cell.min_m  = z_min[idx];
                    cell.max_m  = z_max[idx];
                    cell.mean_m = static_cast<float>(z_sum[idx] / z_cnt[idx]);
                    cell.count  = z_cnt[idx];
                } else {
                    cell.min_m  = tesis_nav::kNaN;
                    cell.max_m  = tesis_nav::kNaN;
                    cell.mean_m = tesis_nav::kNaN;
                    cell.count  = 0;
                }
            }
        }

        grid_pub_->publish(grid);

        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
            "nube procesada | puntos=%u | stamp=%u.%u",
            msg->width * msg->height,
            msg->header.stamp.sec, msg->header.stamp.nanosec);
    }

    std::string cloud_topic_;
    tesis_nav::GridConfig cfg_;
    float x_min_, x_max_, y_min_, y_max_;

    size_t cloud_count_ = 0;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<custom_interfaces::msg::DepthGrid>::SharedPtr grid_pub_;
    rclcpp::TimerBase::SharedPtr heartbeat_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleGridEncoder>());
    rclcpp::shutdown();
    return 0;
}
