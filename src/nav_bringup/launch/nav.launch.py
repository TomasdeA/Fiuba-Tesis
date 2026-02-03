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

    # ---- RealSense launch include ----
    realsense_pkg_share = get_package_share_directory("realsense2_camera")
    rs_launch_path = os.path.join(realsense_pkg_share, "launch", "rs_launch.py")

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_launch_path),
        launch_arguments={
            "enable_gyro": "false",
            "enable_accel": "false",
        }.items(),
    )

    depth_to_matrix = Node(
        package="tesis_nav_mapper_cpp",
        executable="depth_to_matrix",
        name="depth_to_matrix",
        output="screen",
    )

    hw_manager = Node(
        package="hardware_manager",
        executable="hardware_manager",
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

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_hw",
            default_value="false",
            description="Start hardware_manager node",
        ),
        DeclareLaunchArgument(
            "use_viz",
            default_value="false",
            description="Start output viewer heatmap node",
        ),

        realsense,
        depth_to_matrix,
        hw_manager,
        viz,
    ])
