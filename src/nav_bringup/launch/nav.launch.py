import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── Launch arguments ──────────────────────────────────
    use_realsense = LaunchConfiguration('use_realsense')
    use_hw = LaunchConfiguration('use_hw')
    use_viz = LaunchConfiguration('use_viz')
    use_rviz = LaunchConfiguration('use_rviz')

    # ── Config file paths ─────────────────────────────────
    depth_to_matrix_cfg = PathJoinSubstitution([
        FindPackageShare('depth_grid_encoder'),
        'config',
        'depth_to_matrix.yaml',
    ])

    haptic_grid_cfg = PathJoinSubstitution([
        FindPackageShare('haptic_grid_generator'),
        'config',
        'params.yaml',
    ])

    local_mapper_rviz = PathJoinSubstitution([
        FindPackageShare('local_mapper'),
        'rviz',
        'depth_projection.rviz',
    ])

    nav_odometry_launch = PathJoinSubstitution([
        FindPackageShare('nav_odometry'),
        'launch',
        'odometry.launch.py',
    ])

    depth_projection_launch = PathJoinSubstitution([
        FindPackageShare('local_mapper'),
        'launch',
        'depth_projection.launch.py',
    ])

    # ── RealSense camera (optional — needs the HW) ───────
    # We defer the import so the launch file doesn't crash when
    # realsense2_camera is not installed on the dev machine.
    realsense_actions = []
    try:
        from ament_index_python.packages import get_package_share_directory
        realsense_pkg_share = get_package_share_directory('realsense2_camera')
        rs_launch_path = os.path.join(
            realsense_pkg_share, 'launch', 'rs_launch.py')

        realsense = GroupAction(
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(rs_launch_path),
                    launch_arguments={
                        'enable_gyro':      'true',
                        'enable_accel':     'true',
                        'enable_depth':     'true',
                        'unite_imu_method': '1',
                        'align_depth.enable': 'true',
                    }.items(),
                ),
            ],
            forwarding=False,
            condition=IfCondition(use_realsense),
        )
        realsense_actions.append(realsense)
    except Exception:
        realsense_actions.append(
            LogInfo(msg='realsense2_camera not found — skipping camera launch'))

    # ── nav_odometry: IMU + estéreo → nav_msgs/Odometry ──
    # Fusiona giroscopio, acelerómetro y tracker estéreo infrarrojo
    # con un filtro complementario de Mahony.
    # Publica nav_odom (nav_msgs/Odometry) que consume local_mapper.
    nav_odometry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav_odometry_launch),
    )

    # ── Local mapper (occupancy grid from depth + odometry) ──
    depth_projection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(depth_projection_launch),
    )

    # ── Depth-to-matrix encoder (depth grid) ──────────────
    depth_to_matrix = Node(
        package='depth_grid_encoder',
        executable='depth_to_matrix',
        name='depth_to_matrix',
        parameters=[
            LaunchConfiguration('params_file'),
        ],
        output='screen',
        remappings=[
            ('depth/image', '/camera/camera/depth/image_rect_raw'),
        ],
    )

    # ── Haptic grid generator ─────────────────────────────
    haptic_grid = Node(
        package='haptic_grid_generator',
        executable='haptic_grid_generator_node',
        name='haptic_grid_generator',
        parameters=[haptic_grid_cfg],
        output='screen',
    )

    # ── Hardware manager (UART → motors) ──────────────────
    hw_manager = Node(
        package='hardware_manager',
        executable='hardware_manager',
        name='hardware_manager',
        output='screen',
        condition=IfCondition(use_hw),
    )

    # ── Output viewer (heatmap) ───────────────────────────
    viz = Node(
        package='output_viewer',
        executable='depth_grid_heatmap',
        name='depth_grid_heatmap',
        output='screen',
        condition=IfCondition(use_viz),
    )

    # ── RViz2 (local_mapper debug view) ───────────────────
    rviz = ExecuteProcess(
        cmd=['rviz2', '-d', local_mapper_rviz],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    # ── Launch description ────────────────────────────────
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'use_realsense',
            default_value='true',
            description='Launch RealSense D435i camera driver',
        ),
        DeclareLaunchArgument(
            'use_hw',
            default_value='false',
            description='Launch hardware_manager (UART motors)',
        ),
        DeclareLaunchArgument(
            'use_viz',
            default_value='false',
            description='Launch output_viewer heatmap',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2 with local_mapper debug view',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=depth_to_matrix_cfg,
            description='Path to depth_to_matrix params YAML',
        ),

        # Nodes — in pipeline order
        *realsense_actions,
        nav_odometry,
        depth_projection,
        depth_to_matrix,
        haptic_grid,
        hw_manager,
        viz,
        rviz,
    ])
