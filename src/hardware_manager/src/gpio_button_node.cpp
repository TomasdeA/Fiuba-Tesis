/// gpio_button_node.cpp
///
/// Reads a physical GPIO button and calls rosbag_controller's
/// set_recording service. Controls an LED based on recording status.
///
/// This node is only launched when use_hw:=true (Raspberry Pi target).
/// The rosbag_controller node is hardware-agnostic and always launched.
///
/// LED semantics:
///   OFF     = idle (not recording)
///   SOLID   = transitioning (STARTING / STOPPING)
///   BLINK   = actively recording

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "hardware_manager/gpio_interface.hpp"

using namespace std::chrono_literals;

namespace hardware_manager
{

    class GpioButtonNode : public rclcpp::Node
    {
    public:
        GpioButtonNode() : Node("gpio_button")
        {
            // ── Parameters ──────────────────────────────────────────────────────────
            gpio_chip_       = declare_parameter<std::string>("gpio_chip", "gpiochip0");
            switch_gpio_     = static_cast<unsigned>(declare_parameter<int>("switch_gpio", 17));
            led_gpio_        = static_cast<unsigned>(declare_parameter<int>("led_gpio", 27));
            poll_period_ms_  = declare_parameter<int>("poll_period_ms", 50);
            debounce_count_  = declare_parameter<int>("debounce_count", 5);
            blink_period_ms_ = declare_parameter<int>("blink_period_ms", 500);
            recorder_service_ = declare_parameter<std::string>(
                "recorder_service", "/rosbag_controller/set_recording");

            // ── GPIO ────────────────────────────────────────────────────────────────
            switch_gpio_if_ = std::make_unique<GpioInterface>(
                gpio_chip_, switch_gpio_, GpioDirection::Input, "bag_button_switch");
            led_gpio_if_ = std::make_unique<GpioInterface>(
                gpio_chip_, led_gpio_, GpioDirection::Output, "bag_button_led", 0);

            RCLCPP_INFO(get_logger(), "GPIO button: switch BCM %u, LED BCM %u",
                        switch_gpio_, led_gpio_);

            // ── ROS interfaces ───────────────────────────────────────────────────────
            service_client_ = create_client<std_srvs::srv::SetBool>(recorder_service_);

            is_recording_sub_ = create_subscription<std_msgs::msg::Bool>(
                "/rosbag_controller/is_recording",
                rclcpp::QoS(1).transient_local(),
                std::bind(&GpioButtonNode::is_recording_cb, this, std::placeholders::_1));

            poll_timer_ = create_wall_timer(
                std::chrono::milliseconds(poll_period_ms_),
                std::bind(&GpioButtonNode::poll_cb, this));

            blink_timer_ = create_wall_timer(
                std::chrono::milliseconds(blink_period_ms_),
                std::bind(&GpioButtonNode::blink_cb, this));
            blink_timer_->cancel();

            RCLCPP_INFO(get_logger(), "gpio_button ready. Calling: %s",
                        recorder_service_.c_str());
        }

        ~GpioButtonNode() override { set_led(0); }

    private:
        // ── LED modes ────────────────────────────────────────────────────────────
        enum class LedMode { IDLE, TRANSITION, RECORDING };

        // ── GPIO poll + debounce ─────────────────────────────────────────────────
        void poll_cb()
        {
            int raw = 0;
            try
            {
                raw = switch_gpio_if_->read();
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                                      "GPIO read error: %s", e.what());
                return;
            }

            if (raw == debounced_switch_)
            {
                debounce_counter_ = 0;
            }
            else if (++debounce_counter_ >= debounce_count_)
            {
                debounced_switch_ = raw;
                debounce_counter_ = 0;
                RCLCPP_INFO(get_logger(), "Button state → %d", debounced_switch_);
                call_service(debounced_switch_ == 1);
            }
        }

        // ── Service call ─────────────────────────────────────────────────────────
        void call_service(bool start)
        {
            if (!service_client_->service_is_ready())
            {
                RCLCPP_WARN(get_logger(), "recorder service not available");
                return;
            }

            auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
            req->data = start;

            service_client_->async_send_request(
                req,
                [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future)
                {
                    auto resp = future.get();
                    if (resp->success)
                    {
                        set_led_mode(LedMode::TRANSITION);
                    }
                    else
                    {
                        RCLCPP_WARN(get_logger(), "Service rejected: %s",
                                    resp->message.c_str());
                    }
                });
        }

        // ── is_recording status from rosbag_controller ───────────────────────────
        void is_recording_cb(const std_msgs::msg::Bool::SharedPtr msg)
        {
            if (msg->data)
                set_led_mode(LedMode::RECORDING);
            else
                set_led_mode(LedMode::IDLE);
        }

        // ── LED control ──────────────────────────────────────────────────────────
        void set_led_mode(LedMode mode)
        {
            led_mode_ = mode;
            switch (led_mode_)
            {
            case LedMode::IDLE:
                blink_timer_->cancel();
                set_led(0);
                break;
            case LedMode::TRANSITION:
                blink_timer_->cancel();
                set_led(1); // solid ON
                break;
            case LedMode::RECORDING:
                blink_timer_->reset();
                break;
            }
        }

        void blink_cb()
        {
            if (led_mode_ != LedMode::RECORDING)
                return;
            led_state_ = !led_state_;
            set_led(led_state_ ? 1 : 0);
        }

        void set_led(int value)
        {
            try
            {
                led_gpio_if_->write(value);
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                                      "LED write error: %s", e.what());
            }
        }

        // ── Members ───────────────────────────────────────────────────────────────
        std::string gpio_chip_;
        unsigned switch_gpio_{17};
        unsigned led_gpio_{27};
        int poll_period_ms_{50};
        int debounce_count_{5};
        int blink_period_ms_{500};
        std::string recorder_service_;

        std::unique_ptr<GpioInterface> switch_gpio_if_;
        std::unique_ptr<GpioInterface> led_gpio_if_;

        int debounced_switch_{0};
        int debounce_counter_{0};
        bool led_state_{false};
        LedMode led_mode_{LedMode::IDLE};

        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr service_client_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_recording_sub_;
        rclcpp::TimerBase::SharedPtr poll_timer_;
        rclcpp::TimerBase::SharedPtr blink_timer_;
    };

} // namespace hardware_manager

// ── main ──────────────────────────────────────────────────────────────────────
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    try
    {
        rclcpp::spin(std::make_shared<hardware_manager::GpioButtonNode>());
    }
    catch (const std::exception &e)
    {
        RCUTILS_LOG_FATAL_NAMED("gpio_button", "Fatal: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
