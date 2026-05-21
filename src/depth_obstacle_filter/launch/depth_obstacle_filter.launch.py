from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml
import os


def _read_sensor_range():
    """Lee sensor_range.yaml de nav_bringup. Usado como fallback en uso standalone."""
    try:
        from ament_index_python.packages import get_package_share_directory
        sr_file = os.path.join(
            get_package_share_directory('nav_bringup'), 'config', 'sensor_range.yaml')
        with open(sr_file) as f:
            sr = yaml.safe_load(f)
        return float(sr['depth_min_m']), float(sr['depth_max_m'])
    except Exception:
        return 0.25, 5.0


def generate_launch_description():
    pkg_share = FindPackageShare('depth_obstacle_filter').find('depth_obstacle_filter')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)

    ros_params = params['depth_obstacle_filter']['ros__parameters']
    hw_depth = ros_params.get('depth_image_topic',
                              '/camera/camera/depth/image_rect_raw')
    hw_info  = ros_params.get('camera_info_topic',
                              '/camera/camera/depth/camera_info')
    # hw_imu  = ros_params.get('imu_topic', '/camera/camera/imu')  # LEGACY: E4

    _default_min, _default_max = _read_sensor_range()

    def _make_node(context, *args, **kwargs):
        depth_min = float(context.launch_configurations.get(
            'sensor_depth_min_m', str(_default_min)))
        depth_max = float(context.launch_configurations.get(
            'sensor_depth_max_m', str(_default_max)))
        node = Node(
            package='depth_obstacle_filter',
            executable='depth_obstacle_filter_node',
            name='depth_obstacle_filter',
            parameters=[
                params_file,
                {
                    'range_min_m': depth_min,
                    'range_max_m': depth_max,
                    'debug': context.launch_configurations.get('debug', 'false').lower() == 'true',
                    'publish_local_mapper_interface': context.launch_configurations.get(
                        'publish_local_mapper_interface', 'false').lower() == 'true',
                },
            ],
            remappings=[
                ('depth/image',       hw_depth),
                ('depth/camera_info', hw_info),
                # ('imu', hw_imu),  # LEGACY: E4
            ],
            output='screen',
        )
        return [node]

    return LaunchDescription([
        DeclareLaunchArgument(
            'sensor_depth_min_m',
            default_value=str(_default_min),
            description='Distancia mínima válida del sensor [m] (fuente: sensor_range.yaml)',
        ),
        DeclareLaunchArgument(
            'sensor_depth_max_m',
            default_value=str(_default_max),
            description='Distancia máxima válida del sensor [m] (fuente: sensor_range.yaml)',
        ),
        DeclareLaunchArgument(
            'debug',
            default_value='false',
            description='Publicar topics debug no vitales',
        ),
        DeclareLaunchArgument(
            'publish_local_mapper_interface',
            default_value='false',
            description='Publicar free_endpoints y sensor_pos para local_mapper',
        ),
        OpaqueFunction(function=_make_node),
    ])
