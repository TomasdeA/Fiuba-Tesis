#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <string>
#include <vector>

#include "custom_interfaces/msg/depth_cell_stats.hpp"
#include "custom_interfaces/msg/depth_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"

namespace {

constexpr float kPi = 3.14159265358979323846f;

float deg2rad(float deg) { return deg * kPi / 180.0f; }
float rad2deg(float rad) { return rad * 180.0f / kPi; }

float normalizeAngle(float a)
{
    while (a > kPi) a -= 2.0f * kPi;
    while (a < -kPi) a += 2.0f * kPi;
    return a;
}

bool validDepth(float value, float min_m, float max_m)
{
    return std::isfinite(value) && value >= min_m && value <= max_m;
}

float yawFromOdom(const nav_msgs::msg::Odometry& odom)
{
    const auto& q = odom.pose.pose.orientation;

    // Vector forward óptico (0,0,1) rotado por q, proyectado al plano XZ.
    const double fx = 2.0 * (q.x * q.z + q.w * q.y);
    const double fz = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    return std::atan2(static_cast<float>(fx), static_cast<float>(fz));
}

int angleToCol(float angle_rad, float fov_rad, int cols)
{
    const float min_a = -0.5f * fov_rad;
    const float u = (angle_rad - min_a) / fov_rad;
    const int col = static_cast<int>(std::floor(u * static_cast<float>(cols)));
    return (col >= 0 && col < cols) ? col : -1;
}

float colCenterAngle(int col, float fov_rad, int cols)
{
    const float min_a = -0.5f * fov_rad;
    return min_a + (static_cast<float>(col) + 0.5f) * fov_rad /
        static_cast<float>(cols);
}

}  // namespace

class ExtendedDepthGridNode : public rclcpp::Node {
public:
    ExtendedDepthGridNode() : Node("extended_depth_grid")
    {
        input_grid_topic_ = declare_parameter<std::string>(
            "input_grid_topic", "/perception/depth_grid");
        odom_topic_ = declare_parameter<std::string>("odom_topic", "/nav_odom");
        output_grid_topic_ = declare_parameter<std::string>(
            "output_grid_topic", "/perception/extended_depth_grid");
        odom_convention_ = declare_parameter<std::string>(
            "odom_convention", "optical_y_down");
        aperture_command_topic_ = declare_parameter<std::string>(
            "aperture_command_topic", "/perception/depth_grid/aperture_deg");

        input_fov_rad_ = deg2rad(static_cast<float>(
            declare_parameter<double>("input_fov_deg", 90.0)));
        output_fov_rad_ = deg2rad(static_cast<float>(
            declare_parameter<double>("output_fov_deg", 270.0)));
        output_rows_ = declare_parameter<int>("output_rows", 5);
        output_cols_ = declare_parameter<int>("output_cols", 18);
        min_valid_depth_m_ = static_cast<float>(
            declare_parameter<double>("min_valid_depth_m", 0.25));
        max_valid_depth_m_ = static_cast<float>(
            declare_parameter<double>("max_valid_depth_m", 5.0));

        history_ttl_s_ = declare_parameter<double>("history_ttl_s", 2.0);
        max_history_points_ = declare_parameter<int>("max_history_points", 2500);
        lateral_angle_margin_rad_ = deg2rad(static_cast<float>(
            declare_parameter<double>("lateral_angle_margin_deg", 2.0)));

        max_odom_age_s_ = declare_parameter<double>("max_odom_age_s", 0.20);
        max_odom_jump_m_ = static_cast<float>(
            declare_parameter<double>("max_odom_jump_m", 0.60));
        max_odom_jump_yaw_rad_ = deg2rad(static_cast<float>(
            declare_parameter<double>("max_odom_jump_yaw_deg", 35.0)));
        clear_on_odom_jump_ = declare_parameter<bool>("clear_on_odom_jump", true);
        publish_current_when_odom_missing_ =
            declare_parameter<bool>("publish_current_when_odom_missing", true);
        perf_log_enabled_ = declare_parameter<bool>("perf_log_enabled", false);

        if (output_rows_ <= 0 || output_cols_ <= 0) {
            throw std::runtime_error("output_rows/output_cols deben ser positivos");
        }
        if (input_fov_rad_ <= 0.0f || output_fov_rad_ <= input_fov_rad_) {
            throw std::runtime_error("output_fov_deg debe ser mayor que input_fov_deg");
        }
        if (odom_convention_ != "optical_y_down" &&
            odom_convention_ != "rep103") {
            throw std::runtime_error(
                "odom_convention debe ser optical_y_down o rep103");
        }

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, rclcpp::SensorDataQoS(),
            std::bind(&ExtendedDepthGridNode::onOdom, this, std::placeholders::_1));

        grid_sub_ = create_subscription<custom_interfaces::msg::DepthGrid>(
            input_grid_topic_, 10,
            std::bind(&ExtendedDepthGridNode::onGrid, this, std::placeholders::_1));

        aperture_sub_ = create_subscription<std_msgs::msg::Float32>(
            aperture_command_topic_, 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                if (!std::isfinite(msg->data) || msg->data <= 0.0f) return;
                input_fov_rad_ = 2.0f * deg2rad(msg->data);
            });

        grid_pub_ = create_publisher<custom_interfaces::msg::DepthGrid>(
            output_grid_topic_, 10);

        RCLCPP_INFO(get_logger(),
            "ExtendedDepthGrid listo. in=%s odom=%s (%s) out=%s "
            "FOV %.1f -> %.1f deg grid=%dx%d",
            input_grid_topic_.c_str(), odom_topic_.c_str(),
            odom_convention_.c_str(),
            output_grid_topic_.c_str(),
            rad2deg(input_fov_rad_), rad2deg(output_fov_rad_),
            output_rows_, output_cols_);
    }

private:
    struct Pose2D {
        float x = 0.0f;
        float z = 0.0f;
        float yaw = 0.0f;
        rclcpp::Time stamp;
        bool valid = false;
    };

    struct HistoryCell {
        float world_x = 0.0f;
        float world_z = 0.0f;
        int row = 0;
        custom_interfaces::msg::DepthCellStats stats;
        rclcpp::Time stamp;
    };

    void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        Pose2D pose;
        if (odom_convention_ == "rep103") {
            // RTAB-Map: mundo REP-103 (X adelante, Y izquierda, Z arriba)
            // y child frame óptico. Convertimos al plano interno X derecha,
            // Z adelante usado por DepthGrid.
            pose.x = -static_cast<float>(msg->pose.pose.position.y);
            pose.z =  static_cast<float>(msg->pose.pose.position.x);

            const auto& q = msg->pose.pose.orientation;
            const float forward_ros_x = static_cast<float>(
                2.0 * (q.x * q.z + q.w * q.y));
            const float forward_ros_y = static_cast<float>(
                2.0 * (q.y * q.z - q.w * q.x));
            pose.yaw = std::atan2(-forward_ros_y, forward_ros_x);
        } else {
            pose.x = static_cast<float>(msg->pose.pose.position.x);
            pose.z = static_cast<float>(msg->pose.pose.position.z);
            pose.yaw = yawFromOdom(*msg);
        }
        pose.stamp = msg->header.stamp;
        pose.valid = true;

        if (latest_pose_.valid) {
            const float dx = pose.x - latest_pose_.x;
            const float dz = pose.z - latest_pose_.z;
            const float dist = std::sqrt(dx * dx + dz * dz);
            const float dyaw = std::abs(normalizeAngle(pose.yaw - latest_pose_.yaw));
            if (dist > max_odom_jump_m_ || dyaw > max_odom_jump_yaw_rad_) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "Salto de odometría: d=%.3fm dyaw=%.1fdeg. %s",
                    dist, rad2deg(dyaw),
                    clear_on_odom_jump_ ? "Memoria lateral reiniciada." : "Memoria conservada.");
                if (clear_on_odom_jump_) history_.clear();
            }
        }

        latest_pose_ = pose;
    }

    void onGrid(const custom_interfaces::msg::DepthGrid::SharedPtr msg)
    {
        ++frames_in_;
        const auto now = get_clock()->now();
        custom_interfaces::msg::DepthGrid out;
        out.header = msg->header;
        out.rows = output_rows_;
        out.cols = output_cols_;
        out.cells.assign(static_cast<std::size_t>(output_rows_ * output_cols_),
                         custom_interfaces::msg::DepthCellStats{});

        const bool pose_ok = currentPoseUsable(*msg);
        if (!pose_ok) {
            if (publish_current_when_odom_missing_) {
                fillCurrentOnly(*msg, out);
                grid_pub_->publish(out);
            }
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Sin odometría usable para extender grilla; publicando solo FOV actual=%s",
                publish_current_when_odom_missing_ ? "true" : "false");
            return;
        }

        expireHistory(now);
        insertCurrentMeasurements(*msg, latest_pose_);
        composeExtendedGrid(*msg, latest_pose_, out);
        grid_pub_->publish(out);

        if (perf_log_enabled_) {
            RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
                "frames=%zu history=%zu out=%dx%d", frames_in_, history_.size(),
                out.rows, out.cols);
        }
    }

    bool currentPoseUsable(const custom_interfaces::msg::DepthGrid& grid)
    {
        if (!latest_pose_.valid) return false;
        if (max_odom_age_s_ <= 0.0) return true;

        const rclcpp::Time grid_stamp(grid.header.stamp);
        const rclcpp::Time reference =
            grid_stamp.nanoseconds() > 0 ? grid_stamp : get_clock()->now();
        const double age_s = std::abs((reference - latest_pose_.stamp).seconds());
        return age_s <= max_odom_age_s_;
    }

    void fillCurrentOnly(
        const custom_interfaces::msg::DepthGrid& in,
        custom_interfaces::msg::DepthGrid& out) const
    {
        for (int row = 0; row < out.rows; ++row) {
            for (int col = 0; col < out.cols; ++col) {
                const float angle = colCenterAngle(col, output_fov_rad_, out.cols);
                if (std::abs(angle) > 0.5f * input_fov_rad_) continue;
                copyInputCellAtAngle(in, row, angle, out, col);
            }
        }
    }

    void insertCurrentMeasurements(
        const custom_interfaces::msg::DepthGrid& grid,
        const Pose2D& pose)
    {
        const int rows = std::min(grid.rows, output_rows_);
        const int expected = grid.rows * grid.cols;
        if (grid.rows <= 0 || grid.cols <= 0 ||
            static_cast<int>(grid.cells.size()) < expected) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "DepthGrid inválida: rows=%d cols=%d cells=%zu",
                grid.rows, grid.cols, grid.cells.size());
            return;
        }

        for (int row = 0; row < rows; ++row) {
            for (int col = 0; col < grid.cols; ++col) {
                const auto& cell = grid.cells[static_cast<std::size_t>(
                    row * grid.cols + col)];
                if (cell.count <= 0 ||
                    !validDepth(cell.min_m, min_valid_depth_m_, max_valid_depth_m_)) {
                    continue;
                }

                const float rel_angle = colCenterAngle(col, input_fov_rad_, grid.cols);
                const float world_angle = pose.yaw + rel_angle;
                HistoryCell h;
                h.world_x = pose.x + cell.min_m * std::sin(world_angle);
                h.world_z = pose.z + cell.min_m * std::cos(world_angle);
                h.row = row;
                h.stats = cell;
                h.stamp = grid.header.stamp;
                if (rclcpp::Time(h.stamp).nanoseconds() == 0) {
                    h.stamp = get_clock()->now();
                }
                history_.push_back(h);
            }
        }

        while (static_cast<int>(history_.size()) > max_history_points_) {
            history_.pop_front();
        }
    }

    void composeExtendedGrid(
        const custom_interfaces::msg::DepthGrid& current,
        const Pose2D& pose,
        custom_interfaces::msg::DepthGrid& out)
    {
        fillCurrentOnly(current, out);

        std::vector<double> best_age(
            static_cast<std::size_t>(out.rows * out.cols),
            std::numeric_limits<double>::max());

        const rclcpp::Time reference(out.header.stamp);
        const rclcpp::Time ref_time =
            reference.nanoseconds() > 0 ? reference : get_clock()->now();

        for (const auto& h : history_) {
            const float dx = h.world_x - pose.x;
            const float dz = h.world_z - pose.z;
            const float range = std::sqrt(dx * dx + dz * dz);
            if (!validDepth(range, min_valid_depth_m_, max_valid_depth_m_)) continue;

            const float rel_angle = normalizeAngle(std::atan2(dx, dz) - pose.yaw);
            if (std::abs(rel_angle) <= 0.5f * input_fov_rad_ + lateral_angle_margin_rad_) {
                continue;
            }
            if (std::abs(rel_angle) > 0.5f * output_fov_rad_) continue;

            const int col = angleToCol(rel_angle, output_fov_rad_, out.cols);
            if (col < 0 || h.row < 0 || h.row >= out.rows) continue;

            const auto idx = static_cast<std::size_t>(h.row * out.cols + col);
            const double age = std::abs((ref_time - rclcpp::Time(h.stamp)).seconds());
            if (age >= best_age[idx]) continue;

            auto cell = h.stats;
            cell.min_m = range;
            cell.mean_m = range;
            cell.max_m = range;
            cell.count = std::max(1, cell.count);
            out.cells[idx] = cell;
            best_age[idx] = age;
        }
    }

    void copyInputCellAtAngle(
        const custom_interfaces::msg::DepthGrid& in,
        int out_row,
        float angle,
        custom_interfaces::msg::DepthGrid& out,
        int out_col) const
    {
        if (in.rows <= 0 || in.cols <= 0) return;
        const int in_row = std::min(out_row, in.rows - 1);
        const int in_col = angleToCol(angle, input_fov_rad_, in.cols);
        if (in_col < 0) return;

        const int in_idx = in_row * in.cols + in_col;
        if (in_idx < 0 || in_idx >= static_cast<int>(in.cells.size())) return;

        const int out_idx = out_row * out.cols + out_col;
        if (out_idx < 0 || out_idx >= static_cast<int>(out.cells.size())) return;
        out.cells[static_cast<std::size_t>(out_idx)] =
            in.cells[static_cast<std::size_t>(in_idx)];
    }

    void expireHistory(const rclcpp::Time& now)
    {
        if (history_ttl_s_ <= 0.0) {
            history_.clear();
            return;
        }
        while (!history_.empty()) {
            const double age = (now - rclcpp::Time(history_.front().stamp)).seconds();
            if (age <= history_ttl_s_) break;
            history_.pop_front();
        }
    }

    std::string input_grid_topic_;
    std::string odom_topic_;
    std::string output_grid_topic_;
    std::string odom_convention_;
    std::string aperture_command_topic_;

    float input_fov_rad_ = deg2rad(90.0f);
    float output_fov_rad_ = deg2rad(270.0f);
    int output_rows_ = 5;
    int output_cols_ = 18;
    float min_valid_depth_m_ = 0.25f;
    float max_valid_depth_m_ = 5.0f;
    double history_ttl_s_ = 2.0;
    int max_history_points_ = 2500;
    float lateral_angle_margin_rad_ = deg2rad(2.0f);
    double max_odom_age_s_ = 0.20;
    float max_odom_jump_m_ = 0.60f;
    float max_odom_jump_yaw_rad_ = deg2rad(35.0f);
    bool clear_on_odom_jump_ = true;
    bool publish_current_when_odom_missing_ = true;
    bool perf_log_enabled_ = false;

    Pose2D latest_pose_;
    std::deque<HistoryCell> history_;
    std::size_t frames_in_ = 0;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<custom_interfaces::msg::DepthGrid>::SharedPtr grid_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr aperture_sub_;
    rclcpp::Publisher<custom_interfaces::msg::DepthGrid>::SharedPtr grid_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExtendedDepthGridNode>());
    rclcpp::shutdown();
    return 0;
}
