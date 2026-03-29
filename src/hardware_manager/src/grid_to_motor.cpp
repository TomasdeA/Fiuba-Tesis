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
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default: return B115200;
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
  if (!(d_m > 0.0f)) {  // NaN o <=0
    return 0;
  }
  if (d_m <= z_min_m) return 100;
  if (d_m >= z_max_m) return 0;

  float t = (z_max_m - d_m) / (z_max_m - z_min_m);  // 0..1
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
    topic_ = this->declare_parameter<std::string>("topic", "/haptics/intensity_grid");

    // Distance->duty mapping
    z_min_m_ = static_cast<float>(this->declare_parameter<double>("z_min_m", 0.25));
    z_max_m_ = static_cast<float>(this->declare_parameter<double>("z_max_m", 0.8));

    // Throttling (para no saturar UART)
    send_period_ms_ = this->declare_parameter<int>("send_period_ms", 50);

    // Expected grid shape
    expected_rows_ = this->declare_parameter<int>("expected_rows", 1);
    expected_cols_ = this->declare_parameter<int>("expected_cols", 10);

    duty_scale_ = static_cast<float>(this->declare_parameter<double>("duty_scale", 0.4));

    open_uart();

    sub_ = this->create_subscription<custom_interfaces::msg::DepthGrid>(
      topic_, 10,
      std::bind(&DepthGridToUart::on_grid, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(),
      "DepthGridToUart listo. Sub=%s UART=%s@%d grid=%dx%d z_min=%.2f z_max=%.2f",
      topic_.c_str(), port_.c_str(), baud_, expected_rows_, expected_cols_, z_min_m_, z_max_m_);
  }

  ~DepthGridToUart() override
  {
    if (fd_ >= 0) {
      close(fd_);
      fd_ = -1;
    }
  }

private:
  void open_uart()
  {
    fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
      throw std::runtime_error("No pude abrir " + port_ + ": " + std::strerror(errno));
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
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

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      throw std::runtime_error("tcsetattr failed: " + std::string(std::strerror(errno)));
    }

    RCLCPP_INFO(get_logger(), "UART abierto %s @ %d", port_.c_str(), baud_);
  }

  void on_grid(const custom_interfaces::msg::DepthGrid::SharedPtr msg)
  {
    // throttle
    const auto now = this->now();
    if ((now - last_send_time_).nanoseconds() < static_cast<int64_t>(send_period_ms_) * 1000000LL) {
      return;
    }
    last_send_time_ = now;

    const int rows = msg->rows;
    const int cols = msg->cols;

    if (rows != expected_rows_ || cols != expected_cols_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Grid shape inesperada: %dx%d (esperada %dx%d). No envio.",
        rows, cols, expected_rows_, expected_cols_);
      return;
    }

    const int expected = rows * cols; // 10
    if (static_cast<int>(msg->cells.size()) < expected) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "cells.size()=%zu < %d. No envio.",
        msg->cells.size(), expected);
      return;
    }

    // Armar "L d0 d1 ... d9\n"
    std::string line;
    line.reserve(4 + expected * 4);
    line += "L";

    for (int i = 0; i < expected; ++i) {
      const auto & cell = msg->cells[static_cast<size_t>(i)];

      int duty = 0;
      if (cell.count > 0) {
        duty = distance_to_duty(cell.min_m, z_min_m_, z_max_m_);
      }
      duty = static_cast<int>(duty * duty_scale_ + 0.5f);
      duty = clamp_int(duty, 0, 100);
      line += " ";
      line += std::to_string(duty);
    }
    line += "\n";

    ssize_t n = write(fd_, line.data(), line.size());
    if (n < 0) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
        "UART write error: %s", std::strerror(errno));
      return;
    }

    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, "TX -> %s", line.c_str());
  }

  // Params
  std::string port_;
  int baud_{115200};
  std::string topic_;

  float z_min_m_{0.25f};
  float z_max_m_{0.80f};

  int send_period_ms_{50};
  int expected_rows_{1};
  int expected_cols_{10};

  float duty_scale_{0.4f};

  // UART
  int fd_{-1};

  // ROS
  rclcpp::Subscription<custom_interfaces::msg::DepthGrid>::SharedPtr sub_;
  rclcpp::Time last_send_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<DepthGridToUart>());
  } catch (const std::exception & e) {
    fprintf(stderr, "Fatal: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
