from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    pkg_share = FindPackageShare('depth_grid_encoder').find('depth_grid_encoder')
    config_path = os.path.join(pkg_share, 'config', 'extended_depth_grid.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=config_path,
            description='YAML de parámetros para extended_depth_grid',
        ),
        Node(
            package='depth_grid_encoder',
            executable='extended_depth_grid',
            name='extended_depth_grid',
            parameters=[LaunchConfiguration('params_file')],
            output='screen',
        ),
    ])
