/// rosbag_controller_node.cpp
///
/// ROS 2 node that manages ros2 bag record lifecycle.
/// Hardware-agnostic: start/stop is triggered exclusively via the
/// ~/set_recording service (std_srvs/srv/SetBool).
///
/// State machine:
///   IDLE      → set_recording(true)  → STARTING
///   STARTING  → bag alive            → RECORDING
///   RECORDING → set_recording(false) → STOPPING
///   STOPPING  → bag dead             → IDLE
///   RECORDING → bag crash            → IDLE  (failure recovery)
///
/// Status:
///   ~/is_recording (std_msgs/Bool) — true only while state == RECORDING.
///   Used by gpio_button_node (or any other consumer) for LED feedback.
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
#include <string>
#include <vector>

// POSIX process management
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

namespace rosbag_controller
{

    // ─── State machine ────────────────────────────────────────────────────────────
    enum class State { IDLE, STARTING, RECORDING, STOPPING };

    static const char *state_name(State s)
    {
        switch (s)
        {
        case State::IDLE:      return "IDLE";
        case State::STARTING:  return "STARTING";
        case State::RECORDING: return "RECORDING";
        case State::STOPPING:  return "STOPPING";
        }
        return "UNKNOWN";
    }

    // ─── Default topics ───────────────────────────────────────────────────────────
    static const std::vector<std::string> DEFAULT_TOPICS = {
        "/camera/camera/aligned_depth_to_color/image_raw",
        "/camera/camera/aligned_depth_to_color/camera_info",
        "/camera/camera/depth/image_rect_raw",
        "/camera/camera/depth/camera_info",
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
    class RosbagControllerNode : public rclcpp::Node
    {
    public:
        RosbagControllerNode() : Node("rosbag_controller")
        {
            // ── Parameters ──────────────────────────────────────────────────────────
            bag_output_dir_ = declare_parameter<std::string>("bag_output_dir", "~/bags");
            stop_timeout_s_ = declare_parameter<int>("stop_timeout_s", 10);
            topics_ = declare_parameter<std::vector<std::string>>("topics", DEFAULT_TOPICS);

            if (!bag_output_dir_.empty() && bag_output_dir_[0] == '~')
            {
                const char *home = std::getenv("HOME");
                if (home)
                    bag_output_dir_ = std::string(home) + bag_output_dir_.substr(1);
            }

            RCLCPP_INFO(get_logger(), "═══════════════════════════════════════════════");
            RCLCPP_INFO(get_logger(), " rosbag_controller starting");
            RCLCPP_INFO(get_logger(), "  output dir : %s", bag_output_dir_.c_str());
            RCLCPP_INFO(get_logger(), "  topics     : %zu", topics_.size());
            RCLCPP_INFO(get_logger(), "═══════════════════════════════════════════════");

            try
            {
                std::filesystem::create_directories(bag_output_dir_);
            }
            catch (const std::exception &e)
            {
                RCLCPP_WARN(get_logger(), "Could not create bag dir: %s", e.what());
            }

            // ── ROS interfaces ───────────────────────────────────────────────────────
            record_service_ = create_service<std_srvs::srv::SetBool>(
                "~/set_recording",
                std::bind(&RosbagControllerNode::set_recording_cb, this,
                          std::placeholders::_1, std::placeholders::_2));

            is_recording_pub_ = create_publisher<std_msgs::msg::Bool>(
                "~/is_recording", rclcpp::QoS(1).transient_local());

            watchdog_timer_ = create_wall_timer(
                2s, std::bind(&RosbagControllerNode::watchdog_cb, this));

            publish_status();
            RCLCPP_INFO(get_logger(), "Ready. Service: ~/set_recording");
        }

        ~RosbagControllerNode() override
        {
            if (bag_pid_ > 0)
            {
                kill(-bag_pid_, SIGINT);
                wait_for_bag_exit(5);
            }
        }

    private:
        // ── Service callback ─────────────────────────────────────────────────────
        void set_recording_cb(
            const std_srvs::srv::SetBool::Request::SharedPtr request,
            std_srvs::srv::SetBool::Response::SharedPtr response)
        {
            const bool start = request->data;

            if (start && state_ != State::IDLE)
            {
                response->success = false;
                response->message = "Already recording or transitioning";
                return;
            }
            if (!start && state_ == State::IDLE)
            {
                response->success = false;
                response->message = "Not recording";
                return;
            }

            transition_to(start ? State::STARTING : State::STOPPING);
            response->success = true;
            response->message = start ? "Recording started" : "Recording stopping";
        }

        // ── State machine ────────────────────────────────────────────────────────
        void transition_to(State next)
        {
            RCLCPP_INFO(get_logger(), "State: %s → %s",
                        state_name(state_), state_name(next));
            state_ = next;
            publish_status();

            switch (state_)
            {
            case State::STARTING:
                start_rosbag();
                break;
            case State::RECORDING:
                RCLCPP_INFO(get_logger(), "Recording active. PID=%d", bag_pid_);
                break;
            case State::STOPPING:
                stop_rosbag();
                break;
            case State::IDLE:
                bag_pid_ = -1;
                RCLCPP_INFO(get_logger(), "Idle. Ready for next recording.");
                break;
            }
        }

        void publish_status()
        {
            std_msgs::msg::Bool msg;
            msg.data = (state_ == State::RECORDING);
            is_recording_pub_->publish(msg);
        }

        // ── Rosbag lifecycle ─────────────────────────────────────────────────────
        void start_rosbag()
        {
            const std::string bag_path = bag_output_dir_ + "/" + make_bag_name();
            RCLCPP_INFO(get_logger(), "Starting bag: %s (%zu topics)",
                        bag_path.c_str(), topics_.size());

            std::vector<std::string> args = {"ros2", "bag", "record", "-o", bag_path};
            for (const auto &t : topics_)
                args.push_back(t);

            std::vector<char *> argv_ptrs;
            argv_ptrs.reserve(args.size() + 1);
            for (auto &s : args)
                argv_ptrs.push_back(s.data());
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
                setpgid(0, 0);
                execvp("ros2", argv_ptrs.data());
                fprintf(stderr, "[rosbag_ctrl] execvp failed: %s\n", std::strerror(errno));
                _exit(127);
            }

            bag_pid_ = pid;
            RCLCPP_INFO(get_logger(), "rosbag spawned, PID=%d", bag_pid_);
        }

        void stop_rosbag()
        {
            if (bag_pid_ <= 0)
            {
                transition_to(State::IDLE);
                return;
            }
            RCLCPP_INFO(get_logger(), "Sending SIGINT to PG %d", bag_pid_);
            if (kill(-bag_pid_, SIGINT) < 0)
                kill(bag_pid_, SIGINT);
            stop_requested_time_ = this->now();
        }

        bool is_bag_running()
        {
            if (bag_pid_ <= 0)
                return false;
            const int result = waitpid(bag_pid_, nullptr, WNOHANG);
            if (result == 0)
                return true;
            bag_pid_ = -1;
            return false;
        }

        void wait_for_bag_exit(int timeout_s)
        {
            for (int i = 0; i < timeout_s * 10; ++i)
            {
                if (!is_bag_running())
                    return;
                usleep(100000);
            }
            if (bag_pid_ > 0)
            {
                kill(-bag_pid_, SIGKILL);
                waitpid(bag_pid_, nullptr, 0);
                bag_pid_ = -1;
            }
        }

        // ── Watchdog (every 2 s) ─────────────────────────────────────────────────
        void watchdog_cb()
        {
            switch (state_)
            {
            case State::STARTING:
                if (is_bag_running())
                    transition_to(State::RECORDING);
                else
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 4000,
                                         "Waiting for rosbag to start (PID=%d)...", bag_pid_);
                break;

            case State::RECORDING:
                if (!is_bag_running())
                {
                    RCLCPP_ERROR(get_logger(), "rosbag died unexpectedly! → IDLE");
                    transition_to(State::IDLE);
                }
                else
                {
                    publish_status(); // keep alive for late subscribers
                }
                break;

            case State::STOPPING:
                if (!is_bag_running())
                {
                    RCLCPP_INFO(get_logger(), "rosbag exited cleanly. → IDLE");
                    transition_to(State::IDLE);
                }
                else
                {
                    const auto elapsed = (this->now() - stop_requested_time_).seconds();
                    if (elapsed > static_cast<double>(stop_timeout_s_))
                    {
                        RCLCPP_WARN(get_logger(), "Forcing SIGKILL after %ds", stop_timeout_s_);
                        kill(-bag_pid_, SIGKILL);
                        waitpid(bag_pid_, nullptr, WNOHANG);
                        bag_pid_ = -1;
                        transition_to(State::IDLE);
                    }
                    else
                    {
                        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                                             "Waiting for stop... (%.1fs / %ds)",
                                             elapsed, stop_timeout_s_);
                    }
                }
                break;

            case State::IDLE:
                break;
            }
        }

        // ── Bag name ─────────────────────────────────────────────────────────────
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
        std::string bag_output_dir_;
        int stop_timeout_s_{10};
        std::vector<std::string> topics_;

        State state_{State::IDLE};
        pid_t bag_pid_{-1};
        rclcpp::Time stop_requested_time_{0, 0, RCL_ROS_TIME};

        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr record_service_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_recording_pub_;
        rclcpp::TimerBase::SharedPtr watchdog_timer_;
    };

} // namespace rosbag_controller

// ── main ──────────────────────────────────────────────────────────────────────
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    try
    {
        rclcpp::spin(std::make_shared<rosbag_controller::RosbagControllerNode>());
    }
    catch (const std::exception &e)
    {
        RCUTILS_LOG_FATAL_NAMED("rosbag_controller", "Fatal: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
