from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare('haptic_grid_generator'),
        'config',
        'params.yaml',
    ])

    return LaunchDescription([
        Node(
            package='haptic_grid_generator',
            executable='haptic_grid_generator_node',
            name='haptic_grid_generator',
            output='screen',
            parameters=[params],
        ),
    ])
