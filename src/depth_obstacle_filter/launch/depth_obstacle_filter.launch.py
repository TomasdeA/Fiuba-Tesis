from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import yaml
import os


def generate_launch_description():
    config_path = PathJoinSubstitution([
        FindPackageShare('depth_obstacle_filter'),
        'config',
        'params.yaml',
    ])

    pkg_share = FindPackageShare('depth_obstacle_filter').find('depth_obstacle_filter')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)

    ros_params = params['depth_obstacle_filter']['ros__parameters']
    hw_depth = ros_params.get('depth_image_topic',
                              '/camera/camera/depth/image_rect_raw')
    hw_info  = ros_params.get('camera_info_topic',
                              '/camera/camera/depth/camera_info')
    hw_imu   = ros_params.get('imu_topic',
                              '/camera/camera/imu')

    depth_obstacle_filter_node = Node(
        package='depth_obstacle_filter',
        executable='depth_obstacle_filter_node',
        name='depth_obstacle_filter',
        parameters=[config_path],
        remappings=[
            ('depth/image',       hw_depth),
            ('depth/camera_info', hw_info),
            ('imu',               hw_imu),
        ],
        output='screen',
    )

    return LaunchDescription([
        depth_obstacle_filter_node,
    ])
