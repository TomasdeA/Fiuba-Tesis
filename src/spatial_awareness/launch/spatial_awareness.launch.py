import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('spatial_awareness'),
        'config',
        'params.yaml',
    )
    return LaunchDescription([
        Node(
            package='spatial_awareness',
            executable='spatial_awareness_node',
            name='spatial_awareness',
            parameters=[config],
            output='screen',
        ),
    ])
