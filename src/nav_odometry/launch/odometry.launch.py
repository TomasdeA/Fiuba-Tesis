from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import yaml
import os


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

    odometry_node = Node(
        package='nav_odometry',
        executable='odometry_node',
        name='nav_odometry',
        parameters=[config_path],
        remappings=[
            ('gyro',         hw_gyro),
            ('accel',        hw_accel),
            ('color',        hw_color),
            ('camera_info',  hw_camera_info),
            ('depth',        hw_depth),
        ],
        output='screen',
    )

    return LaunchDescription([odometry_node])
