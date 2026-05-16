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
    pkg_share   = FindPackageShare('nav_odometry').find('nav_odometry')
    config_path = os.path.join(pkg_share, 'config', 'params.yaml')

    with open(config_path, 'r') as f:
        params = yaml.safe_load(f)

    ros_params = params['nav_odometry']['ros__parameters']

    # Topics del hardware configurados en params.yaml
    hw_gyro         = ros_params.get('gyro_topic',
                                     '/camera/camera/gyro/sample')
    hw_accel        = ros_params.get('accel_topic',
                                     '/camera/camera/accel/sample')
    hw_color        = ros_params.get('color_topic',
                                     '/camera/camera/color/image_raw')
    hw_camera_info  = ros_params.get('camera_info_topic',
                                     '/camera/camera/color/camera_info')
    hw_depth        = ros_params.get('depth_topic',
                                     '/camera/camera/aligned_depth_to_color/image_raw')

    _default_min, _default_max = _read_sensor_range()

    def _make_node(context, *args, **kwargs):
        depth_min = float(context.launch_configurations.get(
            'sensor_depth_min_m', str(_default_min)))
        depth_max = float(context.launch_configurations.get(
            'sensor_depth_max_m', str(_default_max)))
        use_visual = context.launch_configurations.get(
            'use_visual_odometry', 'false')
        node = Node(
            package='nav_odometry',
            executable='odometry_node',
            name='nav_odometry',
            parameters=[
                config_path,
                {'depth_min_m': depth_min, 'depth_max_m': depth_max,
                 'use_visual_odometry': use_visual.lower() == 'true'},
            ],
            remappings=[
                ('gyro',         hw_gyro),
                ('accel',        hw_accel),
                ('color',        hw_color),
                ('camera_info',  hw_camera_info),
                ('depth',        hw_depth),
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
            'use_visual_odometry',
            default_value='false',
            description='Habilitar tracker RGBD (visual odometry). Si false, solo IMU inercial.',
        ),
        OpaqueFunction(function=_make_node),
    ])
