from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def _read_sensor_range_max():
    """Lee sensor_range.yaml de nav_bringup. Usado como fallback en uso standalone."""
    try:
        import yaml
        from ament_index_python.packages import get_package_share_directory
        sr_file = os.path.join(
            get_package_share_directory('nav_bringup'), 'config', 'sensor_range.yaml')
        with open(sr_file) as f:
            sr = yaml.safe_load(f)
        return float(sr['depth_max_m'])
    except Exception:
        return 5.0


def generate_launch_description():
    pkg_share   = FindPackageShare('local_mapper').find('local_mapper')
    config_path = os.path.join(pkg_share, 'config', 'params.yaml')

    _default_max = _read_sensor_range_max()

    def _make_node(context, *args, **kwargs):
        depth_max = float(context.launch_configurations.get(
            'sensor_depth_max_m', str(_default_max)))
        node = Node(
            package='local_mapper',
            executable='occupancy_mapper_node',
            name='local_mapper',
            parameters=[
                config_path,
                {'occupancy_max_range_m': depth_max},
            ],
            output='screen',
        )
        return [node]

    return LaunchDescription([
        DeclareLaunchArgument(
            'sensor_depth_max_m',
            default_value=str(_default_max),
            description='Distancia máxima válida del sensor [m] (fuente: sensor_range.yaml)',
        ),
        OpaqueFunction(function=_make_node),
    ])
