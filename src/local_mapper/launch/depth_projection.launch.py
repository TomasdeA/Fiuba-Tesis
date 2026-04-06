from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import yaml
import os


def generate_launch_description():
    config_path = PathJoinSubstitution([
        FindPackageShare('local_mapper'),
        'config',
        'params.yaml'
    ])

    # Leer los topics del hardware desde params.yaml para armar el remapping.
    # El nodo usa nombres genéricos (depth/image, depth/camera_info);
    # el launch remapea desde los topics reales configurados en params.yaml.
    pkg_share = FindPackageShare('local_mapper').find('local_mapper')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)

    ros_params = params['local_mapper']['ros__parameters']
    hw_depth = ros_params.get('depth_image_topic',
                              '/camera/camera/depth/image_rect_raw')
    hw_info = ros_params.get('camera_info_topic',
                             '/camera/camera/depth/camera_info')
    hw_imu = ros_params.get('imu_topic',
                            '/camera/camera/imu')

    depth_projection_node = Node(
        package='local_mapper',
        executable='depth_projection_node',
        name='local_mapper',
        parameters=[config_path],
        remappings=[
            ('depth/image', hw_depth),
            ('depth/camera_info', hw_info),
            ('imu', hw_imu),
        ],
        output='screen',
    )

    return LaunchDescription([
        depth_projection_node,
    ])
