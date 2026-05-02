#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/depth_grid.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

static speed_t to_speed(int baud)
{
    switch (baud)
    {
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    case 230400:
        return B230400;
    default:
        return B115200;
    }
}

static int clamp_int(int v, int lo, int hi)
{
    return std::max(lo, std::min(v, hi));
}

// Cerca (<= z_min) -> 100
// Lejos (>= z_max) -> 0
static int distance_to_duty(float d_m, float z_min_m, float z_max_m)
{
    if (!(d_m > 0.0f))
    { // NaN o <=0
        return 0;
    }
    if (d_m <= z_min_m)
        return 100;
    if (d_m >= z_max_m)
        return 0;

    float t = (z_max_m - d_m) / (z_max_m - z_min_m); // 0..1
    int duty = static_cast<int>(t * 100.0f + 0.5f);
    return clamp_int(duty, 0, 100);
}

class DepthGridToUart : public rclcpp::Node
{
public:
    DepthGridToUart() : Node("depth_grid_to_uart")
    {
        // UART params
        port_ = this->declare_parameter<std::string>("port", "/dev/ttyACM0");
        baud_ = this->declare_parameter<int>("baud", 115200);

        // ROS topic
        topic_ = this->declare_parameter<std::string>("topic", "/depth_grid");

        // Distance->duty mapping
        z_min_m_ = static_cast<float>(this->declare_parameter<double>("z_min_m", 0.25));
        z_max_m_ = static_cast<float>(this->declare_parameter<double>("z_max_m", 0.8));
        duty_max_ = this->declare_parameter<int>("duty_max", 70);

        // Throttling (para no saturar UART)
        send_period_ms_ = this->declare_parameter<int>("send_period_ms", 50);

        // Expected grid shape
        expected_rows_ = this->declare_parameter<int>("expected_rows", 5);
        expected_cols_ = this->declare_parameter<int>("expected_cols", 10);

        sub_ = this->create_subscription<custom_interfaces::msg::DepthGrid>(
            topic_, 10,
            std::bind(&DepthGridToUart::on_grid, this, std::placeholders::_1));

        // Intentar abrir UART; si falla, reintentar cada 2s
        if (!try_open_uart())
        {
            RCLCPP_WARN(get_logger(), "UART %s no disponible. Reintentando cada 2s...", port_.c_str());
            reconnect_timer_ = this->create_wall_timer(
                std::chrono::seconds(2),
                std::bind(&DepthGridToUart::reconnect_cb, this));
        }
        else
        {
            RCLCPP_INFO(
                get_logger(),
                "DepthGridToUart listo. Sub=%s UART=%s@%d grid=%dx%d z_min=%.2f z_max=%.2f",
                topic_.c_str(), port_.c_str(), baud_, expected_rows_, expected_cols_, z_min_m_, z_max_m_);
        }
    }

    ~DepthGridToUart() override
    {
        if (fd_ >= 0)
        {
            // Apagar todos los motores antes de cerrar
            std::string stop = "M";
            for (int i = 0; i < expected_rows_ * expected_cols_; ++i)
                stop += " 0";
            stop += "\n";
            write(fd_, stop.data(), stop.size());
            RCLCPP_INFO(get_logger(), "Shutdown: motores apagados.");

            close(fd_);
            fd_ = -1;
        }
    }

private:
    bool try_open_uart()
    {
        fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0)
        {
            return false;
        }

        termios tty{};
        if (tcgetattr(fd_, &tty) != 0)
        {
            throw std::runtime_error("tcgetattr failed: " + std::string(std::strerror(errno)));
        }

        // 8N1
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        // raw
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
        tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
        tty.c_oflag &= ~OPOST;

        // timeouts
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 1; // 0.1s

        speed_t spd = to_speed(baud_);
        cfsetispeed(&tty, spd);
        cfsetospeed(&tty, spd);

        if (tcsetattr(fd_, TCSANOW, &tty) != 0)
        {
            close(fd_);
            fd_ = -1;
            return false;
        }

        RCLCPP_INFO(get_logger(), "UART abierto %s @ %d", port_.c_str(), baud_);
        return true;
    }

    void reconnect_cb()
    {
        if (try_open_uart())
        {
            reconnect_timer_->cancel();
            reconnect_timer_.reset();
            RCLCPP_INFO(
                get_logger(),
                "DepthGridToUart listo. Sub=%s UART=%s@%d grid=%dx%d z_min=%.2f z_max=%.2f",
                topic_.c_str(), port_.c_str(), baud_, expected_rows_, expected_cols_, z_min_m_, z_max_m_);
        }
        else
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                                 "UART %s aun no disponible...", port_.c_str());
        }
    }

    void on_grid(const custom_interfaces::msg::DepthGrid::SharedPtr msg)
    {
        if (fd_ < 0)
            return; // UART aun no disponible

        // throttle
        const auto now = this->now();
        if ((now - last_send_time_).nanoseconds() <
            static_cast<int64_t>(send_period_ms_) * 1000000LL)
        {
            return;
        }
        last_send_time_ = now;

        const int rows = msg->rows;
        const int cols = msg->cols;

        if (rows != expected_rows_ || cols != expected_cols_)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Grid shape inesperada: %dx%d (esperada %dx%d). No envio.",
                rows, cols, expected_rows_, expected_cols_);
            return;
        }

        const int expected = rows * cols; // 50 para 5x10
        if (static_cast<int>(msg->cells.size()) < expected)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "cells.size()=%zu < %d. No envio.",
                msg->cells.size(), expected);
            return;
        }

        // Protocolo UART:
        // "M d0 d1 ... d49\n"
        // - M = mensaje de motores
        // - d0..d49 = duties [0..100] en column-major de derecha a izquierda:
        //   d_index = (cols-1-col)*rows + row
        //   => col=9,row=0 -> d0 (sup-der); col=0,row=4 -> d49 (inf-izq)
        std::string line;
        line.reserve(4 + expected * 4);
        line += "M";

        for (int c = cols - 1; c >= 0; --c)
        {
            for (int r = 0; r < rows; ++r)
            {
                const auto &cell = msg->cells[static_cast<size_t>(r * cols + c)];

                int duty = 0;
                if (cell.count > 0)
                {
                    duty = distance_to_duty(cell.min_m, z_min_m_, z_max_m_);
                    duty = duty * duty_max_ / 100;
                }

                line += " ";
                line += std::to_string(duty);
            }
        }

        line += "\n";

        ssize_t n = write(fd_, line.data(), line.size());
        if (n < 0)
        {
            RCLCPP_ERROR_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "UART write error: %s", std::strerror(errno));
            return;
        }

        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, "TX -> %s", line.c_str());

        // Log cada 2s: primeros 10 valores y el duty máximo para diagnóstico
        int max_duty = *std::max_element(
            std::next(line.begin(), 2), // saltar "M "
            line.end(),
            [](char a, char b)
            { return a < b; }); // comparación char, solo indicativa
        // Calcular max numérico real
        int max_val = 0;
        for (int c = cols - 1; c >= 0; --c)
        {
            for (int r = 0; r < rows; ++r)
            {
                const auto &cell = msg->cells[static_cast<size_t>(r * cols + c)];
                if (cell.count > 0)
                {
                    int d = distance_to_duty(cell.min_m, z_min_m_, z_max_m_);
                    if (d > max_val)
                        max_val = d;
                }
            }
        }
        (void)max_duty;
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                             "TX (duty_max=%d): %s", max_val, line.c_str());
    }

    // Params
    std::string port_;
    int baud_{115200};
    std::string topic_;

    float z_min_m_{0.25f};
    float z_max_m_{0.80f};

    int send_period_ms_{50};
    int expected_rows_{5};
    int expected_cols_{10};

    int duty_max_{70};

    // UART
    int fd_{-1};

    // ROS
    rclcpp::Subscription<custom_interfaces::msg::DepthGrid>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr reconnect_timer_;
    rclcpp::Time last_send_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthGridToUart>());
    rclcpp::shutdown();
    return 0;
}
