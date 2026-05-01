from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("gpio_rosbag_controller")

    default_params = PathJoinSubstitution([pkg_share, "config", "params.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Path to the gpio_rosbag_controller params YAML file.",
        ),

        Node(
            package="gpio_rosbag_controller",
            executable="gpio_rosbag_controller_node",
            name="gpio_rosbag_controller",
            output="screen",
            parameters=[LaunchConfiguration("params_file")],
        ),
    ])
