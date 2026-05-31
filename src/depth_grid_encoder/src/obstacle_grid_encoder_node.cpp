#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/float32.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <time.h>

#include "depth_grid_encoder/depth_utils.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

constexpr float kPi = 3.14159265358979323846f;

// Proyecta una nube de puntos de obstáculos (PointCloud2 en gravity_aligned_frame)
// sobre una grilla 2D XY de rows×cols celdas.
// Cada celda acumula estadísticas de la coordenada Z (profundidad hacia adelante)
// de todos los puntos cuyas coordenadas X,Y caen dentro de la celda.
// En particular, min_m captura el obstáculo más cercano en cada celda.
//
// Eje Y: origen en la cámara (montada en la cabeza del usuario).
//   y_min = -0.20 m  (guardia de seguridad — el obstacle_cloud ya tiene techo removido)
//   y_max = se actualiza desde /depth_obstacle_filter/camera_height (calibración de altura).
//           Valor inicial conservador de 2.20 m hasta recibir la primera calibración.
// El obstacle_cloud ya tiene el suelo y el techo removidos por local_mapper;
// estos límites actúan únicamente como guardia ante puntos espurios.
//
// Métrica de distancia: sqrt(x² + z²) — distancia radial en el plano XZ al cuerpo.
// Se evita z puro para no subestimar obstáculos en celdas laterales.
class ObstacleGridEncoder : public rclcpp::Node
{
public:
    ObstacleGridEncoder() : rclcpp::Node("obstacle_grid_encoder")
    {
        cloud_topic_ = this->declare_parameter<std::string>(
            "cloud_topic", "/depth_obstacle_filter/obstacle_cloud");

        cfg_.rows    = static_cast<int>(this->declare_parameter<int>("rows", 5));
        cfg_.cols    = static_cast<int>(this->declare_parameter<int>("cols", 10));
        cfg_.z_min_m = static_cast<float>(this->declare_parameter<double>("z_min_m", 0.25));
        cfg_.z_max_m = static_cast<float>(this->declare_parameter<double>("z_max_m", 5.0));

        x_min_ = static_cast<float>(this->declare_parameter<double>("x_min_m", -3.0));
        x_max_ = static_cast<float>(this->declare_parameter<double>("x_max_m",  3.0));
        grid_mapping_mode_ = this->declare_parameter<std::string>(
            "grid_mapping_mode", "angular");
        h_angle_min_rad_ = deg2rad(static_cast<float>(
            this->declare_parameter<double>("h_angle_min_deg", -45.0)));
        h_angle_max_rad_ = deg2rad(static_cast<float>(
            this->declare_parameter<double>("h_angle_max_deg", 45.0)));
        y_min_ = static_cast<float>(
            this->declare_parameter<double>("y_min_m", -0.20));
        y_default_max_m_ = static_cast<float>(
            this->declare_parameter<double>("y_default_max_m", 2.20));
        y_margin_m_ = static_cast<float>(
            this->declare_parameter<double>("y_margin_m", 0.10));
        y_max_ = y_default_max_m_;
        if (grid_mapping_mode_ != "angular" && grid_mapping_mode_ != "metric") {
            RCLCPP_WARN(get_logger(),
                "grid_mapping_mode='%s' invalido; usando angular",
                grid_mapping_mode_.c_str());
            grid_mapping_mode_ = "angular";
        }
        perf_log_enabled_ = this->declare_parameter<bool>("perf_log_enabled", false);
        perf_log_period_s_ = this->declare_parameter<double>("perf_log_period_s", 5.0);
        empty_grid_warn_every_ = this->declare_parameter<int>("empty_grid_warn_every", 5);
        hold_last_nonempty_grid_ =
            this->declare_parameter<bool>("hold_last_nonempty_grid", true);
        hold_timeout_ms_ = this->declare_parameter<int>("hold_timeout_ms", 250);

        height_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/depth_obstacle_filter/camera_height",
            rclcpp::QoS(1).transient_local(),
            [this](std_msgs::msg::Float32::SharedPtr msg) {
                const float new_y_max = msg->data + y_margin_m_;
                if (std::abs(new_y_max - y_max_) > 0.01f) {
                    y_max_ = new_y_max;
                    RCLCPP_INFO(get_logger(),
                        "[camera_height] altura=%.3f m -> y_max=%.3f m",
                        msg->data, y_max_);
                }
            });

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
            "Y=[%.2f,%.2f] (guardia seguridad), Z=[%.2f,%.2f], "
            "mode=%s, h=[%.1f,%.1f]deg. "
            "Esperando nube en %s",
            cfg_.rows, cfg_.cols,
            x_min_, x_max_, y_min_, y_max_,
            cfg_.z_min_m, cfg_.z_max_m,
            grid_mapping_mode_.c_str(),
            rad2deg(h_angle_min_rad_), rad2deg(h_angle_max_rad_),
            cloud_topic_.c_str());
        last_perf_log_ = get_clock()->now();
        hold_window_start_ = last_perf_log_;
    }

private:
    struct PerfWindow {
        int frames = 0;
        int empty_grids = 0;
        double sum_total_ms = 0.0;
        double max_total_ms = 0.0;
        double sum_msg_age_ms = 0.0;
        uint64_t sum_points_in = 0;
        uint64_t sum_kept = 0;
        uint64_t sum_drop_invalid = 0;
        uint64_t sum_drop_z = 0;
        uint64_t sum_drop_x = 0;
        uint64_t sum_drop_y = 0;
        uint64_t sum_nonempty_cells = 0;

        void reset() { *this = PerfWindow{}; }
    };

    static double elapsedMs(const struct timespec & start)
    {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        return (now.tv_sec - start.tv_sec) * 1000.0 +
               (now.tv_nsec - start.tv_nsec) / 1e6;
    }

    static float deg2rad(float deg)
    {
        return deg * kPi / 180.0f;
    }

    static float rad2deg(float rad)
    {
        return rad * 180.0f / kPi;
    }

    static int binIndex(float value, float min_value, float max_value, int bins)
    {
        const float range = max_value - min_value;
        if (range <= 0.0f) return -1;
        const int idx = static_cast<int>((value - min_value) / range * bins);
        return (idx >= 0 && idx < bins) ? idx : -1;
    }

    void onCloud(sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        ++cloud_count_;
        struct timespec t0;
        if (perf_log_enabled_) clock_gettime(CLOCK_MONOTONIC, &t0);

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
        const uint64_t points_in = static_cast<uint64_t>(msg->width) *
                                   static_cast<uint64_t>(msg->height);
        uint64_t kept = 0;
        uint64_t drop_invalid = 0;
        uint64_t drop_z = 0;
        uint64_t drop_x = 0;
        uint64_t drop_y = 0;

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
                if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) {
                    ++drop_invalid;
                    continue;
                }

                // Filtro sobre Z (profundidad frontal)
                if (pz < cfg_.z_min_m || pz > cfg_.z_max_m) {
                    ++drop_z;
                    continue;
                }

                if (py < y_min_ || py >= y_max_) {
                    ++drop_y;
                    continue;
                }

                int c = -1;
                int r = -1;
                const int r_raw =
                    static_cast<int>((py - y_min_) / y_range * rows);
                if (r_raw < 0 || r_raw >= rows) continue;
                r = r_raw;

                if (grid_mapping_mode_ == "angular") {
                    // Columnas angulares respecto de la camara.
                    // Las filas siguen siendo franjas metricas de Y.
                    const float h = std::atan2(px, pz);
                    c = binIndex(h, h_angle_min_rad_, h_angle_max_rad_, cols);
                    if (c < 0) {
                        ++drop_x;
                        continue;
                    }
                } else {
                    // Modo historico: franjas metricas X/Y.
                    if (px < x_min_ || px >= x_max_) {
                        ++drop_x;
                        continue;
                    }
                    c = static_cast<int>((px - x_min_) / x_range * cols);
                    if (c < 0 || c >= cols) continue;
                    r = (rows - 1) - r_raw;
                }

                const int idx = r * cols + c;
                // Distancia radial en plano XZ: distancia real de colisión al cuerpo
                const float dist = std::sqrt(px * px + pz * pz);
                if (dist < z_min[idx]) z_min[idx] = dist;
                if (dist > z_max[idx]) z_max[idx] = dist;
                z_sum[idx] += dist;
                ++z_cnt[idx];
                ++kept;
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
        int nonempty_cells = 0;

        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                const int idx = r * cols + c;
                auto & cell = tesis_nav::at(grid, static_cast<size_t>(r), static_cast<size_t>(c));
                if (z_cnt[idx] > 0) {
                    ++nonempty_cells;
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

        const bool empty_grid = (nonempty_cells == 0);
        const auto now_ros_for_hold = get_clock()->now();
        bool published_held_grid = false;
        if (!empty_grid) {
            last_nonempty_grid_ = grid;
            last_nonempty_time_ = now_ros_for_hold;
            has_last_nonempty_grid_ = true;
            held_grid_streak_ = 0;
        } else if (hold_last_nonempty_grid_ && has_last_nonempty_grid_) {
            const double age_ms =
                (now_ros_for_hold - last_nonempty_time_).nanoseconds() / 1e6;
            if (age_ms <= static_cast<double>(hold_timeout_ms_)) {
                auto held = last_nonempty_grid_;
                held.header = grid.header;
                grid_pub_->publish(held);
                published_held_grid = true;
                ++held_grid_streak_;
                ++held_grid_total_;
                ++hold_window_count_;
                if (held_grid_streak_ > held_grid_max_streak_) {
                    held_grid_max_streak_ = held_grid_streak_;
                }
                const double hold_window_s =
                    std::max(1e-3, (now_ros_for_hold - hold_window_start_).seconds());
                const double hold_rate_hz =
                    static_cast<double>(hold_window_count_) / hold_window_s;
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "[grid_hold] grid vacio reemplazado por ultimo no-vacio "
                    "age=%.1fms streak=%d max_streak=%d total=%llu"
                    " window=%d rate=%.2fHz",
                    age_ms, held_grid_streak_, held_grid_max_streak_,
                    static_cast<unsigned long long>(held_grid_total_), hold_window_count_,
                    hold_rate_hz);
                if (hold_window_s >= 10.0) {
                    hold_window_start_ = now_ros_for_hold;
                    hold_window_count_ = 0;
                }
            }
        }

        if (!published_held_grid) {
            grid_pub_->publish(grid);
        }

        if (perf_log_enabled_) {
            perf_window_.frames++;
            perf_window_.sum_points_in += points_in;
            perf_window_.sum_kept += kept;
            perf_window_.sum_drop_invalid += drop_invalid;
            perf_window_.sum_drop_z += drop_z;
            perf_window_.sum_drop_x += drop_x;
            perf_window_.sum_drop_y += drop_y;
            perf_window_.sum_nonempty_cells += static_cast<uint64_t>(nonempty_cells);
        }
        if (empty_grid) {
            if (perf_log_enabled_) ++perf_window_.empty_grids;
            ++empty_grid_streak_;
        } else {
            empty_grid_streak_ = 0;
        }

        if (perf_log_enabled_) {
            const auto now_ros = get_clock()->now();
            const auto msg_stamp = rclcpp::Time(msg->header.stamp);
            const double msg_age_ms = (now_ros - msg_stamp).nanoseconds() / 1e6;
            const double total_ms = elapsedMs(t0);
            perf_window_.sum_msg_age_ms += msg_age_ms;
            perf_window_.sum_total_ms += total_ms;
            perf_window_.max_total_ms = std::max(perf_window_.max_total_ms, total_ms);
        }

        if (empty_grid &&
            (empty_grid_streak_ == 1 || empty_grid_streak_ % empty_grid_warn_every_ == 0)) {
            const auto msg_stamp = rclcpp::Time(msg->header.stamp);
            const double msg_age_ms =
                (get_clock()->now() - msg_stamp).nanoseconds() / 1e6;
            RCLCPP_WARN(get_logger(),
                "[empty_grid] streak=%d points_in=%llu kept=%llu drop_invalid=%llu drop_z=%llu drop_x=%llu drop_y=%llu age=%.1fms",
                empty_grid_streak_,
                static_cast<unsigned long long>(points_in),
                static_cast<unsigned long long>(kept),
                static_cast<unsigned long long>(drop_invalid),
                static_cast<unsigned long long>(drop_z),
                static_cast<unsigned long long>(drop_x),
                static_cast<unsigned long long>(drop_y),
                msg_age_ms);
        }

        if (perf_log_enabled_ &&
            (get_clock()->now() - last_perf_log_).seconds() >= perf_log_period_s_ &&
            perf_window_.frames > 0) {
            const auto now_ros = get_clock()->now();
            const double nf = static_cast<double>(perf_window_.frames);
            RCLCPP_INFO(get_logger(),
                "[PERF %.1fs] frames=%d empty=%d avg_total=%.2fms max_total=%.2fms avg_age=%.1fms avg_points_in=%.0f avg_kept=%.0f avg_cells=%.1f drops(inv/z/x/y)=%.0f/%.0f/%.0f/%.0f",
                perf_log_period_s_,
                perf_window_.frames,
                perf_window_.empty_grids,
                perf_window_.sum_total_ms / nf,
                perf_window_.max_total_ms,
                perf_window_.sum_msg_age_ms / nf,
                static_cast<double>(perf_window_.sum_points_in) / nf,
                static_cast<double>(perf_window_.sum_kept) / nf,
                static_cast<double>(perf_window_.sum_nonempty_cells) / nf,
                static_cast<double>(perf_window_.sum_drop_invalid) / nf,
                static_cast<double>(perf_window_.sum_drop_z) / nf,
                static_cast<double>(perf_window_.sum_drop_x) / nf,
                static_cast<double>(perf_window_.sum_drop_y) / nf);
            perf_window_.reset();
            last_perf_log_ = now_ros;
        }

        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
            "nube procesada | puntos=%u | stamp=%u.%u",
            msg->width * msg->height,
            msg->header.stamp.sec, msg->header.stamp.nanosec);
    }

    std::string cloud_topic_;
    tesis_nav::GridConfig cfg_;
    float x_min_, x_max_, y_min_, y_max_;
    std::string grid_mapping_mode_{"angular"};
    float h_angle_min_rad_{-0.7853982f};
    float h_angle_max_rad_{ 0.7853982f};
    float y_default_max_m_ = 2.20f;
    float y_margin_m_ = 0.10f;
    bool perf_log_enabled_ = false;
    double perf_log_period_s_ = 5.0;
    int empty_grid_warn_every_ = 5;
    bool hold_last_nonempty_grid_ = true;
    int hold_timeout_ms_ = 250;

    size_t cloud_count_ = 0;
    int empty_grid_streak_ = 0;
    int held_grid_streak_ = 0;
    int held_grid_max_streak_ = 0;
    int hold_window_count_ = 0;
    uint64_t held_grid_total_ = 0;
    bool has_last_nonempty_grid_ = false;
    custom_interfaces::msg::DepthGrid last_nonempty_grid_;
    rclcpp::Time last_nonempty_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time hold_window_start_{0, 0, RCL_ROS_TIME};
    PerfWindow perf_window_;
    rclcpp::Time last_perf_log_{0, 0, RCL_ROS_TIME};

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr         height_sub_;
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
