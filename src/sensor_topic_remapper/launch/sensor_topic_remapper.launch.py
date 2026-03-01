from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    depth_image_src = LaunchConfiguration('depth_image_src',
                                          default='/camera/camera/depth/image_rect_raw')
    depth_camera_info_src = LaunchConfiguration('depth_camera_info_src',
                                                default='/camera/camera/depth/camera_info')
    accel_src = LaunchConfiguration('accel_src',
                                     default='/camera/camera/accel/sample')
    gyro_src = LaunchConfiguration('gyro_src',
                                    default='/camera/camera/gyro/sample')

    remapper_node = Node(
        package='sensor_topic_remapper',
        executable='sensor_topic_remapper_node',
        name='sensor_topic_remapper',
        parameters=[
            {'depth_image_src': depth_image_src},
            {'depth_camera_info_src': depth_camera_info_src},
            {'accel_src': accel_src},
            {'gyro_src': gyro_src},
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('depth_image_src',
                              default_value='/camera/camera/depth/image_rect_raw'),
        DeclareLaunchArgument('depth_camera_info_src',
                              default_value='/camera/camera/depth/camera_info'),
        DeclareLaunchArgument('accel_src',
                              default_value='/camera/camera/accel/sample'),
        DeclareLaunchArgument('gyro_src',
                              default_value='/camera/camera/gyro/sample'),
        remapper_node,
    ])
