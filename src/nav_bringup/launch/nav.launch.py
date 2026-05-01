import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_hw = LaunchConfiguration("use_hw")
    use_viz = LaunchConfiguration("use_viz")
    use_gpio_recorder = LaunchConfiguration("use_gpio_recorder")

    ws = EnvironmentVariable("WS_PATH")
    default_cfg = [ws, "/src/depth_grid_encoder/config/depth_to_matrix.yaml"]

    # ── RealSense ─────────────────────────────────────────────────────────────
    # Only streams required for VIO + depth experiments are enabled.
    # infra1/infra2 are disabled to save bandwidth on the Raspberry Pi.
    realsense_pkg_share = get_package_share_directory("realsense2_camera")
    rs_launch_path = os.path.join(realsense_pkg_share, "launch", "rs_launch.py")

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_launch_path),
        launch_arguments={
            "enable_gyro":               "true",
            "enable_accel":              "true",
            "enable_depth":              "true",
            "enable_color":              "true",
            "enable_infra1":             "false",
            "enable_infra2":             "false",
            "unite_imu_method":          "1",
            "align_depth.enable":        "true",
            "depth_module.depth_profile": "848x480x15",
            "rgb_camera.color_profile":   "640x480x15",
        }.items(),
    )

    # ── Perception pipeline ───────────────────────────────────────────────────
    depth_to_matrix = Node(
        package="depth_grid_encoder",
        executable="depth_to_matrix",
        name="depth_to_matrix",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    hw_manager = Node(
        package="hardware_manager",
        executable="grid_to_motor",
        name="hardware_manager",
        output="screen",
        condition=IfCondition(use_hw),
    )

    viz = Node(
        package="output_viewer",
        executable="depth_grid_heatmap",
        name="depth_grid_heatmap",
        output="screen",
        condition=IfCondition(use_viz),
    )

    # ── GPIO rosbag controller ────────────────────────────────────────────────
    gpio_recorder_pkg_share = get_package_share_directory("gpio_rosbag_controller")
    gpio_recorder_launch_path = os.path.join(
        gpio_recorder_pkg_share, "launch", "gpio_rosbag_controller.launch.py"
    )

    gpio_recorder = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gpio_recorder_launch_path),
        condition=IfCondition(use_gpio_recorder),
        # No heredar launch_arguments del padre (especialmente params_file)
        launch_arguments={}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_hw",
            default_value="false",
            description="Start hardware_manager node (haptic actuators via UART).",
        ),
        DeclareLaunchArgument(
            "use_viz",
            default_value="false",
            description="Start output viewer heatmap node.",
        ),
        DeclareLaunchArgument(
            "use_gpio_recorder",
            default_value="true",
            description="Start GPIO rosbag controller (physical switch + LED).",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_cfg,
            description="Path to the ROS2 params file (depth_to_matrix.yaml).",
        ),
        realsense,
        depth_to_matrix,
        hw_manager,
        viz,
        gpio_recorder,
    ])
