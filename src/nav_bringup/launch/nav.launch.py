import glob
import os
import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _resolve_bag_path(context, *args, **kwargs):
    """Return the bag play action with a startup delay.
    Guards: only runs when use_bag=true AND use_realsense=false.
    """
    if context.launch_configurations.get('use_realsense', 'true').lower() == 'true':
        return [LogInfo(msg='use_realsense is true — skipping bag playback')]

    bag_path = context.launch_configurations.get('bag_path', '').strip()
    if not bag_path:
        bags_dir = os.path.expanduser('~/bags')
        candidates = sorted(glob.glob(os.path.join(bags_dir, 'field_*')))
        if not candidates:
            return [LogInfo(msg=f'No bags found in {bags_dir}')]
        bag_path = candidates[-1]

    return [
        LogInfo(msg=f'Bag playback scheduled in 10 s: {bag_path}'),
        TimerAction(
            period=10.0,
            actions=[
                LogInfo(msg=f'Playing bag: {bag_path}'),
                ExecuteProcess(
                    cmd=['ros2', 'bag', 'play', '--loop', bag_path],
                    output='screen',
                ),
            ],
        ),
    ]


def _make_realsense_node(context, *args, **kwargs):
    """Create the RealSense node with streams matched to the odometry mode."""
    if context.launch_configurations.get('use_realsense', 'true').lower() != 'true':
        return []

    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory('realsense2_camera')  # raises if not installed
    except Exception:
        return [LogInfo(msg='realsense2_camera not found — skipping camera launch')]

    use_rtabmap_odom = context.launch_configurations.get(
        'orientation_source', 'nav_odom').lower() == 'rtabmap_odom'
    use_visual = (
        context.launch_configurations.get(
            'use_visual_odometry', 'false').lower() == 'true'
        or use_rtabmap_odom
    )
    depth_profile = context.launch_configurations.get(
        'realsense_depth_profile', '640x480x15')
    color_profile = context.launch_configurations.get(
        'realsense_color_profile', '640x480x15')

    params = {
        'enable_gyro': True,
        'enable_accel': True,
        'enable_depth': True,
        'enable_infra1': False,
        'enable_infra2': False,
        'enable_color': use_visual,
        'align_depth.enable': use_visual,
        'unite_imu_method': 1,
        'depth_module.depth_profile': depth_profile,
        'initial_reset': True,
        'reconnect_timeout': 10.0,
    }
    if use_visual:
        params['rgb_camera.color_profile'] = color_profile

    mode = 'RGBD visual odometry' if use_visual else 'IMU-only odometry'
    return [
        LogInfo(msg=f'RealSense stream profile: {mode}, depth={depth_profile}'),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='camera',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[params],
        ),
    ]


def generate_launch_description():
    # ── Launch arguments ──────────────────────────────────
    use_realsense      = LaunchConfiguration('use_realsense')
    use_hw             = LaunchConfiguration('use_hw')
    use_perception     = LaunchConfiguration('use_perception')
    use_visual_odometry = LaunchConfiguration('use_visual_odometry')
    orientation_source = LaunchConfiguration('orientation_source')
    use_local_mapper   = LaunchConfiguration('use_local_mapper')
    debug              = LaunchConfiguration('debug')
    use_viz            = LaunchConfiguration('use_viz')
    use_rviz           = LaunchConfiguration('use_rviz')
    bag_record         = LaunchConfiguration('bag_record')
    hw_port            = LaunchConfiguration('hw_port')
    pipeline_mode      = LaunchConfiguration('pipeline_mode')
    bag_path           = LaunchConfiguration('bag_path')
    use_bag            = LaunchConfiguration('use_bag')
    performance        = LaunchConfiguration('performance')
    monitor_signal     = LaunchConfiguration('monitor_signal')
    use_rtabmap_odom = PythonExpression([
        "'", orientation_source, "' == 'rtabmap_odom'"
    ])

    # ── Config file paths ─────────────────────────────────
    depth_to_matrix_cfg = PathJoinSubstitution([
        FindPackageShare('depth_grid_encoder'),
        'config',
        'depth_to_matrix.yaml',
    ])

    obstacle_grid_cfg = PathJoinSubstitution([
        FindPackageShare('depth_grid_encoder'),
        'config',
        'obstacle_grid_encoder.yaml',
    ])

    # ── Rango de distancia del sensor (fuente única de verdad) ─────────────
    # Se lee aquí y se inyecta con el nombre correcto en cada nodo/sub-launch.
    # Para cambiar el rango, editar solo nav_bringup/config/sensor_range.yaml.
    from ament_index_python.packages import get_package_share_directory as _gpsd_sr
    _sr_file = os.path.join(_gpsd_sr('nav_bringup'), 'config', 'sensor_range.yaml')
    with open(_sr_file) as _f:
        _sr = yaml.safe_load(_f)
    _depth_min_m = float(_sr['depth_min_m'])
    _depth_max_m = float(_sr['depth_max_m'])

    #haptic_grid_cfg = PathJoinSubstitution([
    #    FindPackageShare('haptic_grid_generator'),
    #    'config',
    #    'params.yaml',
    #])

    nav_rviz = PathJoinSubstitution([
        FindPackageShare('output_viewer'),
        'rviz',
        'nav.rviz',
    ])

    nav_odometry_launch = PathJoinSubstitution([
        FindPackageShare('nav_odometry'),
        'launch',
        'odometry.launch.py',
    ])

    depth_obstacle_filter_launch = PathJoinSubstitution([
        FindPackageShare('depth_obstacle_filter'),
        'launch',
        'depth_obstacle_filter.launch.py',
    ])

    local_mapper_launch = PathJoinSubstitution([
        FindPackageShare('local_mapper'),
        'launch',
        'local_mapper.launch.py',
    ])

    # ── RealSense camera (optional — needs the HW) ───────
    # Created at launch runtime so use_visual_odometry can select the streams:
    # IMU-only keeps color/alignment disabled; visual odometry enables RGBD.
    realsense = OpaqueFunction(function=_make_realsense_node)

    # ── IMU orientation for RTAB-Map ─────────────────────
    # RealSense publishes gyro/accel without orientation. With unite_imu_method=1
    # it also publishes /camera/camera/imu, which Madgwick converts into an
    # orientation-bearing sensor_msgs/Imu for rgbd_odometry's IMU prior.
    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',
        condition=IfCondition(use_rtabmap_odom),
        parameters=[{
            'use_mag': False,
            'publish_tf': False,
            'world_frame': 'enu',
            'gain': 0.01,
            'zeta': 0.0,
            'use_topic_names_from_ros_params': True,
        }],
        remappings=[
            ('imu/data_raw', '/camera/camera/imu'),
            ('imu/data', '/imu/data_filtered'),
        ],
    )

    # ── RTAB-Map RGB-D odometry ──────────────────────────
    # Publishes nav_msgs/Odometry remapped to nav_odom so depth_obstacle_filter
    # can consume it through the same interface as the internal nav_odometry.
    rtabmap_odometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        condition=IfCondition(use_rtabmap_odom),
        parameters=[{
            'frame_id': 'camera_color_optical_frame',
            'odom_frame_id': 'odom',
            # depth_obstacle_filter publica la cadena TF usada por el mapper.
            # Evita dos caminos odom->camera_* y ciclos con los TF estáticos
            # publicados por realsense2_camera.
            'publish_tf': False,
            'approx_sync': True,
            'approx_sync_max_interval': 0.08,
            'use_imu': True,
            'wait_imu_to_init': True,
            'Vis/MaxDepth': '4.0',
            'Vis/MinInliers': '15',
            'Vis/FeatureType': '8',
            'Vis/MaxFeatures': '600',
            'OdomF2M/MaxSize': '3000',
            'Odom/Strategy': '0',
            'Odom/ResetCountdown': '0',
            'Odom/GuessMotion': 'true',
            'Odom/FilteringStrategy': '1',
            'Vis/DepthAsMask': 'true',
            'always_check_imu_tf': False,
        }],
        remappings=[
            ('rgb/image', '/camera/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/camera/color/camera_info'),
            ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
            ('imu', '/imu/data_filtered'),
            ('odom', 'nav_odom'),
        ],
    )

    # ── nav_odometry: IMU + estéreo → nav_msgs/Odometry ──
    # Fusiona giroscopio, acelerómetro y tracker estéreo infrarrojo
    # con un filtro complementario de Mahony.
    # Publica nav_odom (nav_msgs/Odometry) consumido por depth_obstacle_filter
    # y local_mapper.
    nav_odometry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav_odometry_launch),
        launch_arguments={
            'sensor_depth_min_m': str(_depth_min_m),
            'sensor_depth_max_m': str(_depth_max_m),
            'use_visual_odometry': use_visual_odometry,
            'performance': performance,
        }.items(),
        condition=IfCondition(PythonExpression([
            "'", use_perception, "' == 'true' and '", pipeline_mode,
            "' == 'filtered' and '", orientation_source, "' == 'nav_odom'"
        ])),
    )

    # ── depth_obstacle_filter: depth + odometry → obstacle_cloud (odom frame) ──
    # Proyecta la imagen de profundidad, alinea con gravedad, detecta el suelo
    # (RANSAC) y publica obstacle_cloud + free_endpoints + sensor_pos.
    depth_obstacle_filter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(depth_obstacle_filter_launch),
        launch_arguments={
            'sensor_depth_min_m': str(_depth_min_m),
            'sensor_depth_max_m': str(_depth_max_m),
            'debug': debug,
            'publish_local_mapper_interface': use_local_mapper,
            'orientation_source': orientation_source,
            'performance': performance,
        }.items(),
        condition=IfCondition(PythonExpression([
            "'", use_perception, "' == 'true' and '", pipeline_mode, "' == 'filtered'"
        ])),
    )

    # ── local_mapper: obstacle_cloud → occupancy_grid ─────────────────────────
    # Consume los tres topics de depth_obstacle_filter (sincronizados por stamp)
    # y construye el mapa de ocupación 2D incremental en frame odom.
    local_mapper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(local_mapper_launch),
        launch_arguments={
            'sensor_depth_max_m': str(_depth_max_m),
        }.items(),
        condition=IfCondition(use_local_mapper),
    )

    # ── Depth-to-matrix encoder (pipeline: raw) ───────────
    # Activo solo cuando pipeline_mode == 'raw' (default).
    # pipeline_mode == 'none' deja apagados todos los encoders de grilla.
    depth_to_matrix = Node(
        package='depth_grid_encoder',
        executable='depth_to_matrix',
        name='depth_to_matrix',
        parameters=[
            LaunchConfiguration('params_file'),
            {'z_min_m': _depth_min_m, 'z_max_m': _depth_max_m},
        ],
        output='screen',
        condition=IfCondition(PythonExpression(["'", pipeline_mode, "' == 'raw'"])),
        remappings=[
            ('depth/image', '/camera/camera/depth/image_rect_raw'),
        ],
    )

    # ── Obstacle grid encoder (pipeline: filtered) ────────
    # Activo solo cuando pipeline_mode == 'filtered'.
    # pipeline_mode == 'none' deja apagados todos los encoders de grilla.
    # Convierte ObstacleCloud (PointCloud2 con ground removal) → DepthGrid
    obstacle_grid = Node(
        package='depth_grid_encoder',
        executable='obstacle_grid_encoder',
        name='obstacle_grid_encoder',
        parameters=[
            obstacle_grid_cfg,
            {
                'z_min_m': _depth_min_m,
                'z_max_m': _depth_max_m,
                'perf_log_enabled': ParameterValue(performance, value_type=bool),
            },
        ],
        output='screen',
        condition=IfCondition(PythonExpression(["'", pipeline_mode, "' == 'filtered'"])),
    )

    # ── Haptic grid generator ─────────────────────────────
    #haptic_grid = Node(
    #    package='haptic_grid_generator',
    #    executable='haptic_grid_generator_node',
    #    name='haptic_grid_generator',
    #    parameters=[haptic_grid_cfg],
    #    output='screen',
    #)

    # ── Hardware manager (UART → motors) ──────────────────
    hw_manager = Node(
        package='hardware_manager',
        executable='grid_to_motor',
        name='hardware_manager',
        output='screen',
        condition=IfCondition(use_hw),
        parameters=[{'port': hw_port, 'z_max_m': 3.0, 'duty_max': 70}],
    )

    # ── Output viewer (heatmap) ───────────────────────────
    viz = Node(
        package='output_viewer',
        executable='depth_grid_heatmap',
        name='depth_grid_heatmap',
        output='screen',
        condition=IfCondition(use_viz),
        parameters=[{
            'flip_rows_for_display': False,
            'h_aperture_deg': 45.0,
            'h_aperture_min_deg': 15.0,
            'h_aperture_max_deg': 70.0,
        }],
    )

    signal_monitor = Node(
        package='output_viewer',
        executable='grid_signal_monitor',
        name='grid_signal_monitor',
        output='screen',
        condition=IfCondition(monitor_signal),
    )

    # ── RViz2 (debug view) ────────────────────────────────
    rviz = ExecuteProcess(
        cmd=['rviz2', '-d', nav_rviz],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    # ── Odometry path (diagnóstico de trayectoria XZ) ─────
    odometry_path = Node(
        package='output_viewer',
        executable='odometry_path',
        name='odometry_path',
        output='screen',
        condition=IfCondition(use_rviz),
    )

    # ── rosbag_controller (always launched) ──────────────────────────────────
    # rosbag_controller_node is hardware-agnostic: start/stop via service.
    # gpio_button_node lives in hardware_manager and runs only with use_hw:=true.
    recorder_actions = []
    gpio_button_actions = []
    try:
        from ament_index_python.packages import get_package_share_directory as _gpsd
        import os as _os

        recorder_pkg_share = _gpsd('rosbag_controller')
        recorder_actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                _os.path.join(recorder_pkg_share, 'launch', 'rosbag_controller.launch.py')),
            condition=IfCondition(PythonExpression([
                "'", use_realsense, "' == 'true' and '", bag_record, "' == 'true'"
            ])),
        ))
    except Exception:
        recorder_actions.append(
            LogInfo(msg='rosbag_controller not found — skipping'))

    try:
        from ament_index_python.packages import get_package_share_directory as _gpsd
        import os as _os

        hw_manager_pkg_share = _gpsd('hardware_manager')
        gpio_button_actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                _os.path.join(hw_manager_pkg_share, 'launch', 'gpio_button.launch.py')),
            condition=IfCondition(PythonExpression([
                "'", use_hw, "' == 'true' and '", use_realsense, "' == 'true' and '", bag_record, "' == 'true'"
            ])),
        ))
    except Exception:
        pass  # gpio_button is optional; hardware_manager may not have launch dir yet

    # ── Launch description ────────────────────────────────
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'use_realsense',
            default_value='true',
            description='Launch RealSense D435i camera driver',
        ),
        DeclareLaunchArgument(
            'realsense_depth_profile',
            default_value='640x480x15',
            description='RealSense depth profile WIDTHxHEIGHTxFPS',
        ),
        DeclareLaunchArgument(
            'realsense_color_profile',
            default_value='640x480x15',
            description='RealSense color profile WIDTHxHEIGHTxFPS when visual odometry is enabled',
        ),
        DeclareLaunchArgument(
            'use_hw',
            default_value='true',
            description='Launch hardware_manager (UART motors) and gpio_button_node',
        ),
        DeclareLaunchArgument(
            'use_perception',
            default_value='false',
            description='Launch nav_odometry and depth_obstacle_filter (disable on viewer-only machines)',
        ),
        DeclareLaunchArgument(
            'use_visual_odometry',
            default_value='false',
            description='Habilitar tracker RGBD en nav_odometry. Si false, solo IMU inercial (roll/pitch).',
        ),
        DeclareLaunchArgument(
            'orientation_source',
            default_value='nav_odom',
            choices=['nav_odom', 'imu_legacy', 'rtabmap_odom'],
            description=(
                "Fuente de orientación/odometría para depth_obstacle_filter: "
                "'nav_odom' usa nav_odometry interno; "
                "'imu_legacy' usa GravityAligner; "
                "'rtabmap_odom' usa rtabmap rgbd_odometry"
            ),
        ),
        DeclareLaunchArgument(
            'use_local_mapper',
            default_value='false',
            description='Launch local_mapper (occupancy grid builder)',
        ),
        DeclareLaunchArgument(
            'debug',
            default_value='false',
            description='Publish non-essential debug topics (raw/aligned/ground/ceiling clouds)',
        ),
        DeclareLaunchArgument(
            'hw_port',
            default_value='/dev/ttyACM0',
            description='Serial port for the haptic actuator ESP32',
        ),
        DeclareLaunchArgument(
            'use_viz',
            default_value='false',
            description='Launch output_viewer heatmap',
        ),
        DeclareLaunchArgument(
            'bag_record',
            default_value='false',
            description='Enable rosbag_controller and gpio_button launch',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch RViz2 with local_mapper debug view',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=depth_to_matrix_cfg,
            description='Path to depth_to_matrix params YAML',
        ),
        DeclareLaunchArgument(
            'pipeline_mode',
            default_value='raw',
            choices=['raw', 'filtered', 'none'],
            description=(
                "Modo de la pipeline de encodificación de grilla: "
                "'raw' usa depth_to_matrix (imagen de profundidad directa), "
                "'filtered' usa obstacle_grid_encoder (ObstacleCloud con ground removal), "
                "'none' no lanza ningún encoder de grilla"
            ),
        ),
        DeclareLaunchArgument(
            'bag_path',
            default_value='',
            description=(
                'Path to a rosbag to play instead of the live camera. '
                'Empty string (default) selects the latest bag in ~/bags. '
                'Only used when use_realsense:=false.'
            ),
        ),
        DeclareLaunchArgument(
            'use_bag',
            default_value='false',
            description='Play a rosbag instead of (or alongside) the live camera.',
        ),
        DeclareLaunchArgument(
            'performance',
            default_value='true',
            description='Enable perception performance logs and accumulators',
        ),
        DeclareLaunchArgument(
            'monitor_signal',
            default_value='false',
            description='Log depth_grid/obstacle_cloud rate, empties and staleness',
        ),

        # Nodes — in pipeline order
        realsense,
        OpaqueFunction(
            function=_resolve_bag_path,
            condition=IfCondition(use_bag),
        ),
        imu_filter,
        TimerAction(period=5.0, actions=[rtabmap_odometry]),
        nav_odometry,
        depth_obstacle_filter,
        local_mapper,
        depth_to_matrix,
        obstacle_grid,
        #haptic_grid,
        hw_manager,
        viz,
        signal_monitor,
        rviz,
        odometry_path,
        *recorder_actions,
        *gpio_button_actions,
    ])
