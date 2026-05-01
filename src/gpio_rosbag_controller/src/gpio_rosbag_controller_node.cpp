/// gpio_rosbag_controller_node.cpp
///
/// Lightweight GPIO-driven rosbag recorder controller for Raspberry Pi.
///
/// State machine:
///   IDLE      → switch ON  → STARTING
///   STARTING  → bag alive  → RECORDING
///   RECORDING → switch OFF → STOPPING
///   STOPPING  → bag dead   → IDLE
///   RECORDING → bag crash  → IDLE  (failure recovery)
///
/// LED semantics:
///   OFF     = IDLE
///   SOLID   = STARTING / STOPPING
///   BLINK   = RECORDING
///
/// Subprocess:
///   ros2 bag record is spawned via fork()/execvp().
///   Stopped via SIGINT to the process group, then waitpid().

#include <csignal>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// POSIX process management
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "gpio_rosbag_controller/gpio_interface.hpp"

using namespace std::chrono_literals;

namespace gpio_rosbag_controller
{

    // ─── State machine ────────────────────────────────────────────────────────────
    enum class State
    {
        IDLE,      ///< Not recording. LED off.
        STARTING,  ///< Switch went ON; spawning rosbag. LED solid.
        RECORDING, ///< Rosbag running. LED blinking.
        STOPPING,  ///< Switch went OFF; waiting for rosbag to exit. LED solid.
    };

    static const char *state_name(State s)
    {
        switch (s)
        {
        case State::IDLE:
            return "IDLE";
        case State::STARTING:
            return "STARTING";
        case State::RECORDING:
            return "RECORDING";
        case State::STOPPING:
            return "STOPPING";
        }
        return "UNKNOWN";
    }

    // ─── Topics recorded by default ───────────────────────────────────────────────
    // These cover all data needed for offline VIO/depth experiments.
    // Metadata and /parameter_events are intentionally excluded.
    static const std::vector<std::string> DEFAULT_TOPICS = {
        "/camera/camera/aligned_depth_to_color/image_raw",
        "/camera/camera/aligned_depth_to_color/camera_info",
        "/camera/camera/color/image_raw",
        "/camera/camera/color/camera_info",
        "/camera/camera/accel/sample",
        "/camera/camera/gyro/sample",
        "/camera/camera/imu",
        "/camera/camera/accel/imu_info",
        "/camera/camera/gyro/imu_info",
        "/camera/camera/extrinsics/depth_to_color",
        "/camera/camera/extrinsics/depth_to_accel",
        "/camera/camera/extrinsics/depth_to_gyro",
        "/tf_static",
        "/rosout",
    };

    // ─── Node ─────────────────────────────────────────────────────────────────────
    class GpioRosbagControllerNode : public rclcpp::Node
    {
    public:
        GpioRosbagControllerNode()
            : Node("gpio_rosbag_controller")
        {
            // ── Declare parameters ──────────────────────────────────────────────────
            gpio_chip_ = declare_parameter<std::string>("gpio_chip", "gpiochip0");
            switch_gpio_ = static_cast<unsigned>(declare_parameter<int>("switch_gpio", 17));
            led_gpio_ = static_cast<unsigned>(declare_parameter<int>("led_gpio", 27));
            bag_output_dir_ = declare_parameter<std::string>("bag_output_dir", "~/bags");
            poll_period_ms_ = declare_parameter<int>("poll_period_ms", 50);
            debounce_count_ = declare_parameter<int>("debounce_count", 5);
            blink_period_ms_ = declare_parameter<int>("blink_period_ms", 500);
            stop_timeout_s_ = declare_parameter<int>("stop_timeout_s", 10);
            topics_ = declare_parameter<std::vector<std::string>>("topics", DEFAULT_TOPICS);

            // Expand ~ in output dir
            if (!bag_output_dir_.empty() && bag_output_dir_[0] == '~')
            {
                const char *home = std::getenv("HOME");
                if (home)
                {
                    bag_output_dir_ = std::string(home) + bag_output_dir_.substr(1);
                }
            }

            RCLCPP_INFO(get_logger(), "═══════════════════════════════════════════════");
            RCLCPP_INFO(get_logger(), " GPIO Rosbag Controller starting");
            RCLCPP_INFO(get_logger(), "  chip        : %s", gpio_chip_.c_str());
            RCLCPP_INFO(get_logger(), "  switch GPIO : BCM %u", switch_gpio_);
            RCLCPP_INFO(get_logger(), "  LED GPIO    : BCM %u", led_gpio_);
            RCLCPP_INFO(get_logger(), "  output dir  : %s", bag_output_dir_.c_str());
            RCLCPP_INFO(get_logger(), "  topics      : %zu", topics_.size());
            RCLCPP_INFO(get_logger(), "═══════════════════════════════════════════════");

            // ── Initialise GPIO ─────────────────────────────────────────────────────
            try
            {
                switch_gpio_if_ = std::make_unique<GpioInterface>(
                    gpio_chip_, switch_gpio_, GpioDirection::Input, "rosbag_ctrl_switch");
                RCLCPP_INFO(get_logger(), "Switch GPIO %u ready (Input)", switch_gpio_);
            }
            catch (const std::exception &e)
            {
                RCLCPP_FATAL(get_logger(), "Cannot open switch GPIO: %s", e.what());
                throw;
            }

            try
            {
                led_gpio_if_ = std::make_unique<GpioInterface>(
                    gpio_chip_, led_gpio_, GpioDirection::Output, "rosbag_ctrl_led", 0);
                RCLCPP_INFO(get_logger(), "LED GPIO %u ready (Output, OFF)", led_gpio_);
            }
            catch (const std::exception &e)
            {
                RCLCPP_FATAL(get_logger(), "Cannot open LED GPIO: %s", e.what());
                throw;
            }

            // ── Ensure output directory exists ─────────────────────────────────────
            try
            {
                std::filesystem::create_directories(bag_output_dir_);
                RCLCPP_INFO(get_logger(), "Bag output dir ensured: %s", bag_output_dir_.c_str());
            }
            catch (const std::exception &e)
            {
                RCLCPP_WARN(get_logger(), "Could not create bag dir: %s", e.what());
            }

            // ── Timers ──────────────────────────────────────────────────────────────
            poll_timer_ = create_wall_timer(
                std::chrono::milliseconds(poll_period_ms_),
                std::bind(&GpioRosbagControllerNode::poll_cb, this));

            blink_timer_ = create_wall_timer(
                std::chrono::milliseconds(blink_period_ms_),
                std::bind(&GpioRosbagControllerNode::blink_cb, this));
            blink_timer_->cancel(); // only active during RECORDING

            watchdog_timer_ = create_wall_timer(
                2s,
                std::bind(&GpioRosbagControllerNode::watchdog_cb, this));

            RCLCPP_INFO(get_logger(), "Node ready. Waiting for switch...");
        }

        ~GpioRosbagControllerNode() override
        {
            // Ensure rosbag is stopped and LED is off on shutdown
            if (bag_pid_ > 0)
            {
                RCLCPP_INFO(get_logger(), "Destructor: sending SIGINT to rosbag PID %d", bag_pid_);
                kill(-bag_pid_, SIGINT);
                wait_for_bag_exit(5);
            }
            set_led(0);
            RCLCPP_INFO(get_logger(), "GPIO Rosbag Controller stopped cleanly.");
        }

    private:
        // ── GPIO poll + debounce + state machine transitions ────────────────────
        void poll_cb()
        {
            int raw = 0;
            try
            {
                raw = switch_gpio_if_->read();
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR_THROTTLE(
                    get_logger(), *get_clock(), 5000,
                    "GPIO read error: %s", e.what());
                return;
            }

            // Simple majority-based debounce counter
            if (raw == debounced_switch_)
            {
                debounce_counter_ = 0;
            }
            else
            {
                debounce_counter_++;
                if (debounce_counter_ >= debounce_count_)
                {
                    debounced_switch_ = raw;
                    debounce_counter_ = 0;
                    RCLCPP_INFO(
                        get_logger(), "Switch state changed → %d", debounced_switch_);
                    on_switch_changed(debounced_switch_);
                }
            }
        }

        void on_switch_changed(int new_value)
        {
            if (new_value == 1 && state_ == State::IDLE)
            {
                transition_to(State::STARTING);
            }
            else if (new_value == 0 &&
                     (state_ == State::RECORDING || state_ == State::STARTING))
            {
                transition_to(State::STOPPING);
            }
        }

        // ── State transitions ────────────────────────────────────────────────────
        void transition_to(State next)
        {
            RCLCPP_INFO(
                get_logger(), "State: %s → %s",
                state_name(state_), state_name(next));
            state_ = next;

            switch (state_)
            {
            case State::STARTING:
                set_led(1); // solid ON during startup
                blink_timer_->cancel();
                start_rosbag();
                // watchdog_cb will detect when bag is alive and move to RECORDING
                break;

            case State::RECORDING:
                // blink_timer_ started after confirming bag is alive (in watchdog_cb)
                blink_timer_->reset();
                set_led(1); // first half of blink
                RCLCPP_INFO(get_logger(), "Recording active. PID=%d", bag_pid_);
                break;

            case State::STOPPING:
                blink_timer_->cancel();
                set_led(1); // solid ON while stopping
                stop_rosbag();
                // watchdog_cb will detect clean exit and move to IDLE
                break;

            case State::IDLE:
                blink_timer_->cancel();
                set_led(0);
                bag_pid_ = -1;
                RCLCPP_INFO(get_logger(), "Idle. Waiting for switch.");
                break;
            }
        }

        // ── Rosbag lifecycle ─────────────────────────────────────────────────────
        void start_rosbag()
        {
            const std::string bag_name = make_bag_name();
            const std::string bag_path = bag_output_dir_ + "/" + bag_name;

            RCLCPP_INFO(get_logger(), "Starting rosbag recording: %s", bag_path.c_str());
            RCLCPP_INFO(get_logger(), "Topics (%zu):", topics_.size());
            for (const auto &t : topics_)
            {
                RCLCPP_INFO(get_logger(), "  %s", t.c_str());
            }

            // Build argv: ros2 bag record -o <path> <topic1> <topic2> ...
            std::vector<std::string> args;
            args.push_back("ros2");
            args.push_back("bag");
            args.push_back("record");
            args.push_back("-o");
            args.push_back(bag_path);
            for (const auto &t : topics_)
            {
                args.push_back(t);
            }

            // Convert to char* array for execvp
            std::vector<char *> argv_ptrs;
            argv_ptrs.reserve(args.size() + 1);
            for (auto &s : args)
            {
                argv_ptrs.push_back(s.data());
            }
            argv_ptrs.push_back(nullptr);

            const pid_t pid = fork();
            if (pid < 0)
            {
                RCLCPP_ERROR(get_logger(), "fork() failed: %s", std::strerror(errno));
                transition_to(State::IDLE);
                return;
            }

            if (pid == 0)
            {
                // ── Child process ───────────────────────────────────────────────────
                // Create new process group so we can SIGINT the whole group
                setpgid(0, 0);
                execvp("ros2", argv_ptrs.data());
                // If we reach here, exec failed
                fprintf(stderr, "[gpio_rosbag_ctrl] execvp failed: %s\n", std::strerror(errno));
                _exit(127);
            }

            // ── Parent ──────────────────────────────────────────────────────────
            bag_pid_ = pid;
            RCLCPP_INFO(get_logger(), "rosbag process spawned, PID=%d", bag_pid_);
        }

        void stop_rosbag()
        {
            if (bag_pid_ <= 0)
            {
                RCLCPP_WARN(get_logger(), "stop_rosbag called but no bag PID. Moving to IDLE.");
                transition_to(State::IDLE);
                return;
            }

            RCLCPP_INFO(
                get_logger(),
                "Sending SIGINT to process group of PID %d", bag_pid_);
            // Negative PID = process group
            if (kill(-bag_pid_, SIGINT) < 0)
            {
                RCLCPP_WARN(
                    get_logger(), "kill(-pgid, SIGINT) failed: %s. Trying direct PID.",
                    std::strerror(errno));
                kill(bag_pid_, SIGINT);
            }

            stop_requested_time_ = this->now();
        }

        bool is_bag_running()
        {
            if (bag_pid_ <= 0)
            {
                return false;
            }
            const int result = waitpid(bag_pid_, nullptr, WNOHANG);
            if (result == 0)
            {
                return true; // still running
            }
            if (result == bag_pid_)
            {
                RCLCPP_INFO(get_logger(), "rosbag PID %d has exited cleanly.", bag_pid_);
                bag_pid_ = -1;
                return false;
            }
            if (result < 0 && errno == ECHILD)
            {
                // No such child (already reaped or never existed)
                bag_pid_ = -1;
                return false;
            }
            return false;
        }

        void wait_for_bag_exit(int timeout_s)
        {
            for (int i = 0; i < timeout_s * 10; ++i)
            {
                if (!is_bag_running())
                {
                    return;
                }
                usleep(100000); // 100ms
            }
            // Force kill as last resort
            if (bag_pid_ > 0)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "rosbag PID %d did not exit cleanly after %ds. Sending SIGKILL.", bag_pid_, timeout_s);
                kill(-bag_pid_, SIGKILL);
                waitpid(bag_pid_, nullptr, 0);
                bag_pid_ = -1;
            }
        }

        // ── Watchdog: called every 2 s ──────────────────────────────────────────
        void watchdog_cb()
        {
            switch (state_)
            {
            case State::STARTING:
            {
                if (is_bag_running())
                {
                    RCLCPP_INFO(get_logger(), "rosbag is alive. Moving to RECORDING.");
                    transition_to(State::RECORDING);
                }
                else
                {
                    RCLCPP_WARN_THROTTLE(
                        get_logger(), *get_clock(), 4000,
                        "Waiting for rosbag to start (PID=%d)...", bag_pid_);
                }
                break;
            }

            case State::RECORDING:
            {
                if (!is_bag_running())
                {
                    RCLCPP_ERROR(
                        get_logger(),
                        "rosbag process died unexpectedly! Returning to IDLE.");
                    transition_to(State::IDLE);
                }
                break;
            }

            case State::STOPPING:
            {
                if (!is_bag_running())
                {
                    RCLCPP_INFO(get_logger(), "rosbag exited cleanly. Moving to IDLE.");
                    transition_to(State::IDLE);
                }
                else
                {
                    const auto elapsed =
                        (this->now() - stop_requested_time_).seconds();
                    if (elapsed > static_cast<double>(stop_timeout_s_))
                    {
                        RCLCPP_WARN(
                            get_logger(),
                            "rosbag did not exit after %ds. Forcing SIGKILL.", stop_timeout_s_);
                        kill(-bag_pid_, SIGKILL);
                        waitpid(bag_pid_, nullptr, WNOHANG);
                        bag_pid_ = -1;
                        transition_to(State::IDLE);
                    }
                    else
                    {
                        RCLCPP_INFO_THROTTLE(
                            get_logger(), *get_clock(), 2000,
                            "Waiting for rosbag to stop... (%.1fs / %ds)", elapsed, stop_timeout_s_);
                    }
                }
                break;
            }

            case State::IDLE:
                // Nothing to watch
                break;
            }
        }

        // ── LED blink timer (active only during RECORDING) ───────────────────────
        void blink_cb()
        {
            if (state_ != State::RECORDING)
            {
                return;
            }
            led_state_ = !led_state_;
            set_led(led_state_ ? 1 : 0);
        }

        // ── LED helper ────────────────────────────────────────────────────────────
        void set_led(int value)
        {
            if (!led_gpio_if_)
            {
                return;
            }
            try
            {
                led_gpio_if_->write(value);
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR_THROTTLE(
                    get_logger(), *get_clock(), 5000,
                    "LED write error: %s", e.what());
            }
        }

        // ── Bag name generation ──────────────────────────────────────────────────
        static std::string make_bag_name()
        {
            std::time_t t = std::time(nullptr);
            std::tm tm{};
            localtime_r(&t, &tm);
            char buf[32];
            std::strftime(buf, sizeof(buf), "field_%Y%m%d_%H%M%S", &tm);
            return std::string(buf);
        }

        // ── Members ───────────────────────────────────────────────────────────────

        // Parameters
        std::string gpio_chip_;
        unsigned int switch_gpio_{17};
        unsigned int led_gpio_{27};
        std::string bag_output_dir_;
        int poll_period_ms_{50};
        int debounce_count_{5};
        int blink_period_ms_{500};
        int stop_timeout_s_{10};
        std::vector<std::string> topics_;

        // GPIO
        std::unique_ptr<GpioInterface> switch_gpio_if_;
        std::unique_ptr<GpioInterface> led_gpio_if_;

        // State machine
        State state_{State::IDLE};

        // Switch debounce
        int debounced_switch_{0};
        int debounce_counter_{0};

        // LED blink
        bool led_state_{false};

        // Rosbag process
        pid_t bag_pid_{-1};
        rclcpp::Time stop_requested_time_{0, 0, RCL_ROS_TIME};

        // Timers
        rclcpp::TimerBase::SharedPtr poll_timer_;
        rclcpp::TimerBase::SharedPtr blink_timer_;
        rclcpp::TimerBase::SharedPtr watchdog_timer_;
    };

} // namespace gpio_rosbag_controller

// ── main ──────────────────────────────────────────────────────────────────────
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    try
    {
        rclcpp::spin(
            std::make_shared<gpio_rosbag_controller::GpioRosbagControllerNode>());
    }
    catch (const std::exception &e)
    {
        RCUTILS_LOG_FATAL_NAMED(
            "gpio_rosbag_controller", "Fatal exception: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
