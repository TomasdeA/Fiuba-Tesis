from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path = PathJoinSubstitution([
        FindPackageShare('local_mapper'),
        'config',
        'params.yaml',
    ])

    occupancy_mapper_node = Node(
        package='local_mapper',
        executable='occupancy_mapper_node',
        name='local_mapper',
        parameters=[config_path],
        output='screen',
    )

    return LaunchDescription([
        occupancy_mapper_node,
    ])
