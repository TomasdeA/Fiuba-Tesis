// ─────────────────────────────────────────────────────────────────────────────
// nav_odometry/odometry_node.cpp
//
// Nodo ROS2 del pipeline VIO.
//
// Suscribe (nombres genéricos; el launch file remapea al hardware):
//   gyro          --> sensor_msgs/Imu   (~200 Hz, solo angular_velocity)
//   accel         --> sensor_msgs/Imu   (~100 Hz, solo linear_acceleration)
//   color         --> sensor_msgs/Image  (~30 fps, rgb8)
//   camera_info   --> sensor_msgs/CameraInfo (latched)
//   depth         --> sensor_msgs/Image  (~30 fps, 16UC1, aligned_depth_to_color)
//
// Publica:
//   nav_odom --> nav_msgs/Odometry   (en frame "odom", hijo "camera_link")
//   /tf      --> geometry_msgs/TransformStamped (odom --> camera_link)
// ─────────────────────────────────────────────────────────────────────────────

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <chrono>

#include "nav_odometry/odometry_estimator.hpp"

using std::placeholders::_1;

// ─────────────────────────────────────────────────────────────────────────────
// Empaquetado de estado en nav_msgs/Odometry.
//
// La convención interna del pipeline (Y abajo, Z adelante — óptica del D435i)
// se publica sin conversión: el único consumidor actual (local_mapper) opera
// en la misma convención, por lo que la conversión es innecesaria.
//
// FUTURO: si se integran nodos externos ROS (Nav2, rtabmap, etc.) que requieran
// REP-103 (Z arriba, X adelante), agregar la conversión de marco aquí:
//   q_ros = q_fix * q_int * q_fix_inv,  q_fix = {w=0, x=1, y=0, z=0}
//   p_ros = {p.x, -p.y, -p.z}
// ─────────────────────────────────────────────────────────────────────────────
static nav_msgs::msg::Odometry packOdometry(
    const nav_odometry::OdometryState& state,
    const std::string& odom_frame,
    const std::string& child_frame,
    const rclcpp::Time& stamp)
{
    const nav_odometry::Vec3&       p = state.pose.position;
    const nav_odometry::Quaternion& q = state.pose.orientation;
    const nav_odometry::Vec3&       v = state.linear_velocity;
    const nav_odometry::Vec3&       w = state.angular_velocity;

    nav_msgs::msg::Odometry msg;
    msg.header.stamp    = stamp;
    msg.header.frame_id = odom_frame;
    msg.child_frame_id  = child_frame;

    msg.pose.pose.position.x    = static_cast<double>(p.x);
    msg.pose.pose.position.y    = static_cast<double>(p.y);
    msg.pose.pose.position.z    = static_cast<double>(p.z);
    msg.pose.pose.orientation.w = static_cast<double>(q.w);
    msg.pose.pose.orientation.x = static_cast<double>(q.x);
    msg.pose.pose.orientation.y = static_cast<double>(q.y);
    msg.pose.pose.orientation.z = static_cast<double>(q.z);

    msg.twist.twist.linear.x  = static_cast<double>(v.x);
    msg.twist.twist.linear.y  = static_cast<double>(v.y);
    msg.twist.twist.linear.z  = static_cast<double>(v.z);
    msg.twist.twist.angular.x = static_cast<double>(w.x);
    msg.twist.twist.angular.y = static_cast<double>(w.y);
    msg.twist.twist.angular.z = static_cast<double>(w.z);

    return msg;
}

// ─────────────────────────────────────────────────────────────────────────────
class OdometryNode : public rclcpp::Node {
public:
    OdometryNode() : rclcpp::Node("odometry_node") {
        declareParameters();
        buildEstimator();
        setupPublishers();
        setupSubscribers();

        RCLCPP_INFO(get_logger(),
            "OdometryNode listo. Backend: %s", estimator_->name());
    }

private:
    // ── Parámetros ────────────────────────────────────────────────────────────
    void declareParameters() {
        // Filtro complementario
        declare_parameter<double>("kp_accel",   2.0);
        declare_parameter<double>("ki_accel",   0.005);
        declare_parameter<double>("alpha_yaw",  0.10);
        declare_parameter<double>("min_visual_confidence", 0.30);
        declare_parameter<double>("gravity_magnitude",     9.807);

        // Tracker estéreo
        declare_parameter<int>   ("fast_threshold",        20);
        declare_parameter<int>   ("max_features",          300);
        declare_parameter<double>("min_feature_dist_px",   15.0);
        declare_parameter<double>("ransac_threshold_px",   1.0);
        declare_parameter<double>("min_inlier_ratio",      0.5);
        declare_parameter<int>   ("min_inliers",           20);
        declare_parameter<double>("max_translation_m",     0.5);
        declare_parameter<double>("depth_min_m",             0.25);
        declare_parameter<double>("depth_max_m",             5.0);
        declare_parameter<int>   ("max_keyframe_age_frames", 15);

        // Estimador
        declare_parameter<double>("max_velocity_mps",      3.0);
        declare_parameter<double>("translation_confidence_scale", 1.0);

        // Frames y topics
        declare_parameter<std::string>("odom_frame",  "odom");
        declare_parameter<std::string>("child_frame", "camera_link");

        // Topics de hardware (el launch file remapea a los nombres genéricos)
        declare_parameter<std::string>("gyro_topic",
            "/camera/camera/gyro/sample");
        declare_parameter<std::string>("accel_topic",
            "/camera/camera/accel/sample");
        declare_parameter<std::string>("color_topic",
            "/camera/camera/color/image_raw");
        declare_parameter<std::string>("camera_info_topic",
            "/camera/camera/color/camera_info");
        declare_parameter<std::string>("depth_topic",
            "/camera/camera/aligned_depth_to_color/image_raw");
        declare_parameter<double>("depth_scale_m", 0.001);
    }

    void buildEstimator() {
        nav_odometry::OdometryEstimator::Config cfg;

        cfg.filter.kp_accel              = static_cast<float>(get_parameter("kp_accel").as_double());
        cfg.filter.ki_accel              = static_cast<float>(get_parameter("ki_accel").as_double());
        cfg.filter.alpha_yaw             = static_cast<float>(get_parameter("alpha_yaw").as_double());
        cfg.filter.min_visual_confidence = static_cast<float>(get_parameter("min_visual_confidence").as_double());
        cfg.filter.gravity_magnitude     = static_cast<float>(get_parameter("gravity_magnitude").as_double());

        cfg.tracker.fast_threshold       = get_parameter("fast_threshold").as_int();
        cfg.tracker.max_features         = get_parameter("max_features").as_int();
        cfg.tracker.min_feature_dist_px  = static_cast<float>(get_parameter("min_feature_dist_px").as_double());
        cfg.tracker.essential_ransac_threshold = get_parameter("ransac_threshold_px").as_double();
        cfg.tracker.min_inlier_ratio     = static_cast<float>(get_parameter("min_inlier_ratio").as_double());
        cfg.tracker.min_inliers          = get_parameter("min_inliers").as_int();
        cfg.tracker.max_translation_per_frame_m =
            static_cast<float>(get_parameter("max_translation_m").as_double());
        cfg.tracker.depth_min_m =
            static_cast<float>(get_parameter("depth_min_m").as_double());
        cfg.tracker.depth_max_m =
            static_cast<float>(get_parameter("depth_max_m").as_double());
        cfg.tracker.max_keyframe_age_frames =
            get_parameter("max_keyframe_age_frames").as_int();

        depth_scale_m_ = static_cast<float>(get_parameter("depth_scale_m").as_double());

        cfg.max_velocity_mps             = static_cast<float>(get_parameter("max_velocity_mps").as_double());
        cfg.translation_confidence_scale = static_cast<float>(get_parameter("translation_confidence_scale").as_double());

        estimator_ = std::make_unique<nav_odometry::OdometryEstimator>(cfg);

        odom_frame_  = get_parameter("odom_frame").as_string();
        child_frame_ = get_parameter("child_frame").as_string();
    }

    void setupPublishers() {
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("nav_odom", 10);
        tf_br_    = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

    void setupSubscribers() {
        const auto qos = rclcpp::SensorDataQoS();

        // Giroscopio y acelerómetro: callbacks independientes (tasas distintas)
        gyro_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "gyro", qos,
            std::bind(&OdometryNode::onGyro, this, _1));

        accel_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "accel", qos,
            std::bind(&OdometryNode::onAccel, this, _1));

        // Camera info (latched)
        info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera_info", rclcpp::QoS(1).reliable(),
            std::bind(&OdometryNode::onCameraInfo, this, _1));

        // Imagen color izquierda
        color_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "color", qos,
            std::bind(&OdometryNode::onColorImage, this, _1));

        // Depth image (siempre activo)
        depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "depth", qos,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                latest_depth_ = msg;
                ++depth_frames_received_;
            });
        RCLCPP_INFO(get_logger(),
            "Depth topic: %s",
            get_parameter("depth_topic").as_string().c_str());

        // Timer de reporte de rendimiento: se dispara cada 5 s.
        perf_timer_ = create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&OdometryNode::logPerfStats, this));
    }

    // ── Callbacks ─────────────────────────────────────────────────────────────

    void onGyro(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // Medido:
        // hz: ~200 msg/s
        // bw: ~68 KB/s
        const auto t0 = std::chrono::steady_clock::now();
        nav_odometry::ImuSample s;
        s.timestamp_s = toSec(msg->header.stamp);
        s.gyro  = {static_cast<float>(msg->angular_velocity.x),
                   static_cast<float>(msg->angular_velocity.y),
                   static_cast<float>(msg->angular_velocity.z)};
        // Fusionar con el último accel cacheado (max 10 ms de desfase).
        // NO se llama processImu desde onAccel para evitar que el timestamp del
        // acelerómetro (100 Hz) pise last_t_ antes de llegar el giro coincidente
        // (200 Hz), lo que causaría dt≈0 en 1 de cada 4 muestras de giro (−25%).
        s.accel = latest_accel_;
        estimator_->processImu(s);
        publishOdometry(msg->header.stamp);
        perf_gyro_.record(us_since(t0));
    }

    void onAccel(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // Medido:
        // hz: ~100 msg/s
        // bw: ~34 KB/s
        const auto t0 = std::chrono::steady_clock::now();
        // Solo cachear: la integración ocurre en onGyro para mantener dt correcto.
        latest_accel_ = {static_cast<float>(msg->linear_acceleration.x),
                         static_cast<float>(msg->linear_acceleration.y),
                         static_cast<float>(msg->linear_acceleration.z)};
        perf_accel_.record(us_since(t0));
    }

    void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        if (intrinsics_set_) return;

        nav_odometry::CameraIntrinsics intr;
        intr.fx     = static_cast<float>(msg->k[0]);
        intr.fy     = static_cast<float>(msg->k[4]);
        intr.cx     = static_cast<float>(msg->k[2]);
        intr.cy     = static_cast<float>(msg->k[5]);
        intr.width  = msg->width;
        intr.height = msg->height;

        intr.valid = (intr.fx > 1.f && intr.fy > 1.f);
        estimator_->setCameraIntrinsics(intr);
        intrinsics_set_ = intr.valid;

        RCLCPP_INFO(get_logger(),
            "Intrínseca cámara: fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
            intr.fx, intr.fy, intr.cx, intr.cy);
    }

    void onColorImage(const sensor_msgs::msg::Image::SharedPtr msg) {
        onRgbdFrame(msg);
    }

    void onRgbdFrame(const sensor_msgs::msg::Image::SharedPtr& color_msg)
    {
        if (!intrinsics_set_) return;

        // Convertir rgb8 --> mono8 para el tracker (FAST+LK operan en gris)
        // El D435i publica siempre rgb8; se asume ese encoding.
        if (color_msg->encoding != "rgb8") {
            RCLCPP_WARN_ONCE(get_logger(),
                "Encoding inesperado: %s (se espera rgb8).",
                color_msg->encoding.c_str());
            return;
        }
        const std::size_t npix = color_msg->width * color_msg->height;
        std::vector<uint8_t> gray_buf(npix);
        const uint8_t* src = color_msg->data.data();
        for (std::size_t i = 0; i < npix; ++i) {
            const uint8_t r = src[3*i + 0];
            const uint8_t g = src[3*i + 1];
            const uint8_t b = src[3*i + 2];
            // BT.601 luma: 0.299R + 0.587G + 0.114B (enteros para velocidad)
            gray_buf[i] = static_cast<uint8_t>((77u*r + 150u*g + 29u*b) >> 8u);
        }
        const uint8_t* gray_ptr = gray_buf.data();

        nav_odometry::ColorFrame color_frame;
        color_frame.timestamp_s = toSec(color_msg->header.stamp);
        color_frame.width       = color_msg->width;
        color_frame.height      = color_msg->height;
        color_frame.step        = color_msg->width;  // paso en gris = width
        color_frame.data        = gray_ptr;

        // Forwarding del depth image al tracker
        if (latest_depth_) {
            nav_odometry::DepthFrame df;
            df.data    = reinterpret_cast<const uint16_t*>(latest_depth_->data.data());
            df.width   = latest_depth_->width;
            df.height  = latest_depth_->height;
            df.step    = latest_depth_->step;
            df.scale_m = depth_scale_m_;
            estimator_->setDepthFrame(df);
        } else {
            RCLCPP_WARN_ONCE(get_logger(),
                "No se ha recibido ningún frame de profundidad. "
                "Verificar remap del topic 'depth'.");
        }

        const auto t0 = std::chrono::steady_clock::now();
        estimator_->processRgbd(color_frame);
        const auto t1 = std::chrono::steady_clock::now();
        publishOdometry(color_msg->header.stamp);
        const auto t2 = std::chrono::steady_clock::now();

        perf_rgbd_process_.record(us_since(t0, t1));
        perf_publish_.record(us_since(t1, t2));
    }

    // ── Reporte de rendimiento ──────────────────────────────────────────────────

    void logPerfStats() {
        const auto est   = estimator_->getPerfReport();
        const auto state = estimator_->getState();

        // Ratio de frames VO válidos (donde se integró traslación)
        const int64_t vo_total = est.tracker.count;
        const int64_t vo_valid = est.visual_update.count;
        const float vo_ratio = (vo_total > 0)
            ? static_cast<float>(vo_valid) / static_cast<float>(vo_total) * 100.f
            : 0.f;

        RCLCPP_INFO(get_logger(),
            "\n"
            "[Odometría /5s] ──────────────────────────────────\n"
            "  Posición actual : x=%.3f  y=%.3f  z=%.3f  [m]\n"
            "  VO válido/total : %ld / %ld frames  (%.0f%%)\n"
            "  Depth recv /5s  : %ld frames\n"
            "  Callbacks (nodo):\n"
            "    onGyro          avg=%6.1f µs  min=%5.1f  max=%6.1f  n=%ld\n"
            "    onAccel         avg=%6.1f µs  min=%5.1f  max=%6.1f  n=%ld\n"
            "    processRgbd     avg=%6.1f µs  min=%5.1f  max=%6.1f  n=%ld\n"
            "    publishOdometry avg=%6.1f µs  min=%5.1f  max=%6.1f  n=%ld\n"
            "  Estimador (interno):\n"
            "    IMU filter      avg=%6.1f µs  min=%5.1f  max=%6.1f  n=%ld\n"
            "    tracker.process avg=%6.1f µs  min=%5.1f  max=%6.1f  n=%ld\n"
            "    visual_update   avg=%6.1f µs  min=%5.1f  max=%6.1f  n=%ld\n"
            "    rgbd_total      avg=%6.1f µs  min=%5.1f  max=%6.1f  n=%ld\n"
            "──────────────────────────────────────────────────",
            static_cast<double>(state.pose.position.x),
            static_cast<double>(state.pose.position.y),
            static_cast<double>(state.pose.position.z),
            vo_valid, vo_total, static_cast<double>(vo_ratio),
            depth_frames_received_,
            perf_gyro_.avg_us(),   perf_gyro_.min_display(),   perf_gyro_.max_display(),   perf_gyro_.count,
            perf_accel_.avg_us(),  perf_accel_.min_display(),  perf_accel_.max_display(),  perf_accel_.count,
            perf_rgbd_process_.avg_us(), perf_rgbd_process_.min_display(), perf_rgbd_process_.max_display(), perf_rgbd_process_.count,
            perf_publish_.avg_us(), perf_publish_.min_display(), perf_publish_.max_display(), perf_publish_.count,
            est.imu_filter.avg_us(),    est.imu_filter.min_display(),    est.imu_filter.max_display(),    est.imu_filter.count,
            est.tracker.avg_us(),       est.tracker.min_display(),       est.tracker.max_display(),       est.tracker.count,
            est.visual_update.avg_us(), est.visual_update.min_display(), est.visual_update.max_display(), est.visual_update.count,
            est.rgbd_total.avg_us(),  est.rgbd_total.min_display(),  est.rgbd_total.max_display(),  est.rgbd_total.count);

        // Resetear contadores para el siguiente intervalo
        perf_gyro_.reset();
        perf_accel_.reset();
        perf_rgbd_process_.reset();
        perf_publish_.reset();
        estimator_->resetPerfReport();
        depth_frames_received_ = 0;
    }

    // ── Publicación ───────────────────────────────────────────────────────────

    void publishOdometry(const rclcpp::Time& stamp) {
        const nav_odometry::OdometryState state = estimator_->getState();
        if (!state.valid) return;

        const auto odom_msg = packOdometry(state, odom_frame_, child_frame_, stamp);
        odom_pub_->publish(odom_msg);

        // Broadcast TF: odom --> child_frame
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header        = odom_msg.header;
        tf_msg.child_frame_id = child_frame_;
        tf_msg.transform.translation.x = odom_msg.pose.pose.position.x;
        tf_msg.transform.translation.y = odom_msg.pose.pose.position.y;
        tf_msg.transform.translation.z = odom_msg.pose.pose.position.z;
        tf_msg.transform.rotation      = odom_msg.pose.pose.orientation;
        tf_br_->sendTransform(tf_msg);
    }

    // ── Utilidades ────────────────────────────────────────────────────────────

    // Tiempo transcurrido en us desde t0.
    static double us_since(const std::chrono::steady_clock::time_point& t0) {
        return std::chrono::duration<double, std::micro>(
            std::chrono::steady_clock::now() - t0).count();
    }
    // Tiempo transcurrido en us entre dos puntos.
    static double us_since(
        const std::chrono::steady_clock::time_point& t0,
        const std::chrono::steady_clock::time_point& t1)
    {
        return std::chrono::duration<double, std::micro>(t1 - t0).count();
    }

    static double toSec(const rclcpp::Time& t) {
        return static_cast<double>(t.nanoseconds()) * 1e-9;
    }

    // ── Miembros ──────────────────────────────────────────────────────────────
    std::unique_ptr<nav_odometry::OdometryEstimator> estimator_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster>        tf_br_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr        gyro_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr        accel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      color_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      depth_sub_;

    sensor_msgs::msg::Image::SharedPtr latest_depth_;  // depth más reciente
    nav_odometry::Vec3 latest_accel_{};                 // último accel cacheado

    std::string odom_frame_;
    std::string child_frame_;
    bool        intrinsics_set_{false};
    float       depth_scale_m_{0.001f};
    int64_t     depth_frames_received_{0};

    // ── Temporización ───────────────────────────────────────────────────────────
    nav_odometry::SectionStats perf_gyro_{};           // onGyro completo
    nav_odometry::SectionStats perf_accel_{};          // onAccel completo
    nav_odometry::SectionStats perf_rgbd_process_{};   // processRgbd en el nodo
    nav_odometry::SectionStats perf_publish_{};        // publishOdometry tras rgbd
    rclcpp::TimerBase::SharedPtr perf_timer_;
};

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryNode>());
    rclcpp::shutdown();
    return 0;
}
