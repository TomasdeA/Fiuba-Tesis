from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_heatmap = LaunchConfiguration('use_heatmap')
    use_signal_monitor = LaunchConfiguration('use_signal_monitor')
    use_rviz = LaunchConfiguration('use_rviz')
    use_odometry_path = LaunchConfiguration('use_odometry_path')
    flip_rows_for_display = LaunchConfiguration('flip_rows_for_display')
    pygame_view_mode = LaunchConfiguration('pygame_view_mode')
    h_angle_min_deg = LaunchConfiguration('h_angle_min_deg')
    h_angle_max_deg = LaunchConfiguration('h_angle_max_deg')

    nav_rviz = PathJoinSubstitution([
        FindPackageShare('output_viewer'),
        'rviz',
        'nav.rviz',
    ])

    heatmap = Node(
        package='output_viewer',
        executable='depth_grid_heatmap',
        name='depth_grid_heatmap',
        output='screen',
        condition=IfCondition(use_heatmap),
        parameters=[{
            'flip_rows_for_display': ParameterValue(
                flip_rows_for_display,
                value_type=bool,
            ),
            'pygame_view_mode': pygame_view_mode,
            'h_angle_min_deg': ParameterValue(
                h_angle_min_deg,
                value_type=float,
            ),
            'h_angle_max_deg': ParameterValue(
                h_angle_max_deg,
                value_type=float,
            ),
        }],
    )

    signal_monitor = Node(
        package='output_viewer',
        executable='grid_signal_monitor',
        name='grid_signal_monitor',
        output='screen',
        condition=IfCondition(use_signal_monitor),
    )

    odometry_path = Node(
        package='output_viewer',
        executable='odometry_path',
        name='odometry_path',
        output='screen',
        condition=IfCondition(use_odometry_path),
    )

    rviz = ExecuteProcess(
        cmd=['rviz2', '-d', nav_rviz],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_heatmap',
            default_value='true',
            description='Launch depth grid heatmap viewer',
        ),
        DeclareLaunchArgument(
            'use_signal_monitor',
            default_value='true',
            description='Launch depth_grid/obstacle_cloud signal monitor',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2 with output_viewer/nav.rviz',
        ),
        DeclareLaunchArgument(
            'use_odometry_path',
            default_value='false',
            description='Launch odometry path diagnostic node',
        ),
        DeclareLaunchArgument(
            'flip_rows_for_display',
            default_value='false',
            description='Flip heatmap rows to match filtered obstacle grid',
        ),
        DeclareLaunchArgument(
            'pygame_view_mode',
            default_value='curved',
            description='Pygame view mode: curved or grid',
        ),
        DeclareLaunchArgument('h_angle_min_deg', default_value='-45.0'),
        DeclareLaunchArgument('h_angle_max_deg', default_value='45.0'),
        heatmap,
        signal_monitor,
        odometry_path,
        rviz,
    ])
