from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("rosbag_controller")

    default_params = PathJoinSubstitution([pkg_share, "config", "recorder_params.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Path to rosbag_controller params YAML.",
        ),
        Node(
            package="rosbag_controller",
            executable="rosbag_controller_node",
            name="rosbag_controller",
            output="screen",
            parameters=[LaunchConfiguration("params_file")],
        ),
    ])
