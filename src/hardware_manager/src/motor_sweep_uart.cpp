#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

using namespace std::chrono_literals;

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

class MotorSweepUart : public rclcpp::Node
{
public:
  MotorSweepUart() : Node("motor_sweep_uart")
  {
    port_ = this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    baud_ = this->declare_parameter<int>("baud", 115200);
    period_s_ = this->declare_parameter<double>("period_s", 1.0);

    duties_ = {0, 25, 50, 75, 100};
    motor_id_ = 0;
    duty_idx_ = 0;

    fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
      throw std::runtime_error("No pude abrir " + port_ + ": " + std::strerror(errno));
    }

    configure_uart(fd_, baud_);
    RCLCPP_INFO(get_logger(), "UART abierto %s @ %d", port_.c_str(), baud_);

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(period_s_),
      std::bind(&MotorSweepUart::tick, this)
    );
  }

  ~MotorSweepUart() override
  {
    if (fd_ >= 0) {
      close(fd_);
      fd_ = -1;
    }
  }

private:
  void configure_uart(int fd, int baud)
  {
    termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
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

    // timeouts (no bloqueante duro)
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1; // 0.1s

    speed_t spd = to_speed(baud);
    cfsetispeed(&tty, spd);
    cfsetospeed(&tty, spd);

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
      throw std::runtime_error("tcsetattr failed: " + std::string(std::strerror(errno)));
    }
  }

  void tick()
  {
    int duty = duties_[duty_idx_];

    std::string cmd = "M " + std::to_string(motor_id_) + " " + std::to_string(duty) + "\n";

    ssize_t n = write(fd_, cmd.data(), cmd.size());
    if (n < 0) {
      RCLCPP_ERROR(get_logger(), "UART write error: %s", std::strerror(errno));
      return;
    }

    RCLCPP_INFO(get_logger(), "TX -> %s", cmd.c_str());

    // avanzar
    duty_idx_++;
    if (duty_idx_ >= static_cast<int>(duties_.size())) {
      duty_idx_ = 0;
      motor_id_++;
      if (motor_id_ >= 48) {
        motor_id_ = 0;
        RCLCPP_INFO(get_logger(), "Ciclo completo 0–47 terminado");
      }
    }
  }

  std::string port_;
  int baud_;
  double period_s_;

  int fd_{-1};

  std::vector<int> duties_;
  int motor_id_{0};
  int duty_idx_{0};

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<MotorSweepUart>());
  } catch (const std::exception & e) {
    fprintf(stderr, "Fatal: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
