#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/haptic_grid.hpp"

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

class HapticGridToUart : public rclcpp::Node
{
public:
    HapticGridToUart() : Node("haptic_grid_to_uart")
    {
        // UART params
        port_ = this->declare_parameter<std::string>("port", "/dev/serial0");
        baud_ = this->declare_parameter<int>("baud", 115200);

        // ROS topic: grilla final de intensidades 0..100.
        topic_ = this->declare_parameter<std::string>("topic", "/perception/haptic_grid");

        duty_max_ = this->declare_parameter<int>("duty_max", 70);

        // Throttling (para no saturar UART)
        send_period_ms_ = this->declare_parameter<int>("send_period_ms", 50);

        // Expected grid shape
        expected_rows_ = this->declare_parameter<int>("expected_rows", 5);
        expected_cols_ = this->declare_parameter<int>("expected_cols", 10);

        haptic_sub_ = this->create_subscription<custom_interfaces::msg::HapticGrid>(
            topic_, 10,
            std::bind(&HapticGridToUart::on_haptic_grid, this, std::placeholders::_1));

        // Intentar abrir UART; si falla, reintentar cada 2s
        if (!try_open_uart())
        {
            RCLCPP_WARN(get_logger(), "UART %s no disponible. Reintentando cada 2s...", port_.c_str());
            reconnect_timer_ = this->create_wall_timer(
                std::chrono::seconds(2),
                std::bind(&HapticGridToUart::reconnect_cb, this));
        }
        else
        {
            RCLCPP_INFO(
                get_logger(),
                "HapticGridToUart listo. Sub=%s UART=%s@%d grid=%dx%d duty_max=%d",
                topic_.c_str(), port_.c_str(), baud_, expected_rows_, expected_cols_, duty_max_);
        }
    }

    ~HapticGridToUart() override
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
    std::vector<std::string> candidate_ports() const
    {
        std::vector<std::string> ports;
        if (!port_.empty())
        {
            ports.push_back(port_);
        }

        auto add_if_missing = [&ports](const std::string &p) {
            if (std::find(ports.begin(), ports.end(), p) == ports.end())
            {
                ports.push_back(p);
            }
        };

        add_if_missing("/dev/serial0");
        add_if_missing("/dev/ttyAMA0");
        add_if_missing("/dev/ttyS0");
        add_if_missing("/dev/ttyACM0");
        add_if_missing("/dev/ttyACM1");

        DIR *dir = opendir("/dev");
        if (dir != nullptr)
        {
            while (dirent *entry = readdir(dir))
            {
                const std::string name(entry->d_name);
                if (name.rfind("ttyACM", 0) == 0)
                {
                    add_if_missing("/dev/" + name);
                }
            }
            closedir(dir);
        }

        return ports;
    }

    bool try_open_uart()
    {
        for (const auto &candidate : candidate_ports())
        {
            const int maybe_fd = open(candidate.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
            if (maybe_fd < 0)
            {
                continue;
            }

            termios tty{};
            if (tcgetattr(maybe_fd, &tty) != 0)
            {
                close(maybe_fd);
                continue;
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

            if (tcsetattr(maybe_fd, TCSANOW, &tty) != 0)
            {
                close(maybe_fd);
                continue;
            }

            fd_ = maybe_fd;
            port_ = candidate;
            RCLCPP_INFO(get_logger(), "UART abierto %s @ %d", port_.c_str(), baud_);
            return true;
        }

        return false;
    }

    void reconnect_cb()
    {
        if (try_open_uart())
        {
            reconnect_timer_->cancel();
            reconnect_timer_.reset();
            RCLCPP_INFO(
                get_logger(),
                "HapticGridToUart listo. Sub=%s UART=%s@%d grid=%dx%d duty_max=%d",
                topic_.c_str(), port_.c_str(), baud_, expected_rows_, expected_cols_, duty_max_);
        }
        else
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                                 "UART %s aun no disponible...", port_.c_str());
        }
    }

    bool should_send_now()
    {
        if (fd_ < 0)
            return false; // UART aun no disponible

        // throttle
        const auto now = this->now();
        if ((now - last_send_time_).nanoseconds() <
            static_cast<int64_t>(send_period_ms_) * 1000000LL)
        {
            return false;
        }
        last_send_time_ = now;
        return true;
    }

    bool validate_shape(int rows, int cols, std::size_t size)
    {
        if (rows != expected_rows_ || cols != expected_cols_)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Grid shape inesperada: %dx%d (esperada %dx%d). No envio.",
                rows, cols, expected_rows_, expected_cols_);
            return false;
        }

        const int expected = rows * cols; // 50 para 5x10
        if (static_cast<int>(size) < expected)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "grid.size()=%zu < %d. No envio.",
                size, expected);
            return false;
        }

        return true;
    }

    void send_duties(const std::vector<int>& duties, int rows, int cols)
    {
        const int expected = rows * cols;

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
                line += " ";
                line += std::to_string(duties[static_cast<std::size_t>(r * cols + c)]);
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
    }

    void on_haptic_grid(const custom_interfaces::msg::HapticGrid::SharedPtr msg)
    {
        if (!should_send_now())
            return;

        const int rows = msg->rows;
        const int cols = msg->cols;
        if (!validate_shape(rows, cols, msg->intensities.size()) ||
            !validate_shape(rows, cols, msg->active.size()))
        {
            return;
        }

        const int expected = rows * cols;
        std::vector<int> duties(static_cast<std::size_t>(expected), 0);
        for (int i = 0; i < expected; ++i)
        {
            const auto idx = static_cast<std::size_t>(i);
            if (msg->active[idx])
            {
                const int intensity = clamp_int(
                    static_cast<int>(msg->intensities[idx] + 0.5f), 0, 100);
                duties[idx] = intensity * duty_max_ / 100;
            }
        }

        send_duties(duties, rows, cols);
    }

    // Params
    std::string port_;
    int baud_{115200};
    std::string topic_;

    int send_period_ms_{50};
    int expected_rows_{5};
    int expected_cols_{10};

    int duty_max_{70};

    // UART
    int fd_{-1};

    // ROS
    rclcpp::Subscription<custom_interfaces::msg::HapticGrid>::SharedPtr haptic_sub_;
    rclcpp::TimerBase::SharedPtr reconnect_timer_;
    rclcpp::Time last_send_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HapticGridToUart>());
    rclcpp::shutdown();
    return 0;
}
