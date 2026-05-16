from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("hardware_manager")

    default_params = PathJoinSubstitution([pkg_share, "config", "gpio_button_params.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Path to gpio_button_node params YAML.",
        ),
        Node(
            package="hardware_manager",
            executable="gpio_button_node",
            name="gpio_button",
            output="screen",
            parameters=[LaunchConfiguration("params_file")],
        ),
    ])
