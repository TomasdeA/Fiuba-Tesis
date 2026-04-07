import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
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
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── Launch arguments ──────────────────────────────────
    use_realsense = LaunchConfiguration('use_realsense')
    use_hw = LaunchConfiguration('use_hw')
    use_viz = LaunchConfiguration('use_viz')
    use_rviz = LaunchConfiguration('use_rviz')
    vio_method = LaunchConfiguration('vio_method')

    # True cuando vio_method requiere rgbd_odometry ("visual_only" o "full")
    use_visual_odom = PythonExpression(
        ["'", vio_method, "' in ('visual_only', 'full')"]
    )

    # ── Config file paths ─────────────────────────────────
    depth_to_matrix_cfg = PathJoinSubstitution([
        FindPackageShare('depth_grid_encoder'),
        'config',
        'depth_to_matrix.yaml',
    ])

    local_mapper_cfg = PathJoinSubstitution([
        FindPackageShare('local_mapper'),
        'config',
        'params.yaml',
    ])

    haptic_grid_cfg = PathJoinSubstitution([
        FindPackageShare('haptic_grid_generator'),
        'config',
        'params.yaml',
    ])

    local_mapper_rviz = PathJoinSubstitution([
        FindPackageShare('local_mapper'),
        'rviz',
        'local_mapper.rviz',
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

        realsense = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_launch_path),
            launch_arguments={
                'enable_gyro':        'true',
                'enable_accel':       'true',
                'unite_imu_method':   '1',
                'pointcloud.enable':  'true',   # habilita /camera/camera/depth/color/points
                # Igualar resolución color a depth para que rtabmap pueda usar DepthAsMask
                'rgb_camera.color_profile': '848x480x30',
            }.items(),
            condition=IfCondition(use_realsense),
        )
        realsense_actions.append(realsense)
    except Exception:
        realsense_actions.append(
            LogInfo(msg='realsense2_camera not found — skipping camera launch'))

    # ── Filtro Madgwick: accel+gyro → IMU con orientación ─
    # El RealSense publica accel y gyro por separado sin orientación.
    # Madgwick los fusiona y produce sensor_msgs/Imu con quaternion válido
    # en /imu/data_filtered, que rgbd_odometry usa como prior de movimiento (VIO).
    #
    # Nota: Madgwick solo consume el tópico .sample (sensor_msgs/Imu).
    # Los tópicos imu_info (matrices de ruido del fabricante) los usa
    # robot_localization/EKF, pero Madgwick fija su propio modelo de ruido
    # vía los parámetros gain y zeta.
    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',
        parameters=[{
            'use_mag':           False,   # sin magnetómetro
            'publish_tf':        False,   # no publicar TF propio
            'world_frame':       'enu',
            'gain':              0.01,    # gain bajo → confía más en gyro (menos drift)
            'zeta':              0.0,
            'use_topic_names_from_ros_params': True,
        }],
        remappings=[
            ('imu/data_raw', '/camera/camera/imu'),   # IMU fusionado del RealSense
            ('imu/data',     '/imu/data_filtered'),   # salida con orientación
        ],
    )

    # ── Visual-Inertial Odometry (VIO) via rtabmap rgbd_odometry ──
    # rgbd_odometry trackea features visuales frame a frame.
    # Con use_imu=true, el IMU filtrado actúa como prior de movimiento:
    #   - En rotaciones rápidas donde el visual falla → el IMU mantiene la pose
    #   - En frames sin suficientes features → IMU propaga la estimación
    # Resultado: odometría mucho más estable que solo visual.
    rgbd_odometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        condition=IfCondition(use_visual_odom),
        parameters=[{
            'frame_id':                 'camera_color_optical_frame',
            'odom_frame_id':            'odom',
            'publish_tf':               True,
            'approx_sync':              True,
            'approx_sync_max_interval': 0.05,
            # IMU como prior — clave para estabilidad en rotaciones
            'use_imu':                  True,
            'wait_imu_to_init':         True,
            # Visual odometry params
            'Vis/MaxDepth':             '4.0',
            'Vis/MinInliers':           '15',    # más restrictivo → menos falsos positivos
            'Vis/FeatureType':          '8',     # GFTT/BRIEF: rápido y estable
            'Vis/MaxFeatures':          '600',
            'OdomF2M/MaxSize':          '3000',
            'Odom/Strategy':            '0',     # Frame-to-Map: acumula referencia
            'Odom/ResetCountdown':      '0',     # no resetear nunca — el IMU mantiene cuando falla visual
            'Odom/GuessMotion':         'true',
            'Odom/FilteringStrategy':   '1',     # filtro kalman sobre la pose odométrica
            # DepthAsMask requiere que color y depth tengan la misma resolución (848x480)
            'Vis/DepthAsMask':          'true',
            # El TF entre camera_imu_optical_frame y camera_color_optical_frame
            # es estático — no necesita re-lookup en cada frame
            'always_check_imu_tf':      False,
        }],
        remappings=[
            # Color (NO depth) para tracking de features visuales:
            # la imagen RGB tiene mas texturas y son mas estables que el
            # mapa de profundidad, que tiene huecos en superficies lisas.
            ('rgb/image',       '/camera/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/camera/color/camera_info'),
            # La profundidad se usa solo para escalar la odometría visual
            # (evitar deriva de escala), no para el tracking de features.
            ('depth/image',     '/camera/camera/depth/image_rect_raw'),
            ('imu',             '/imu/data_filtered'),
        ],
    )

    # ── Local mapper (occupancy grid from depth + IMU) ────
    local_mapper = Node(
        package='local_mapper',
        executable='local_mapper_node',
        name='local_mapper',
        parameters=[local_mapper_cfg, {'vio_method': vio_method}],
        output='screen',
        remappings=[
            ('depth/image',       '/camera/camera/depth/image_rect_raw'),
            ('depth/camera_info', '/camera/camera/depth/camera_info'),
            ('imu/accel',         '/camera/camera/accel/sample'),
            ('imu/gyro',          '/camera/camera/gyro/sample'),
        ],
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
            default_value='false',
            description='Launch RViz2 with local_mapper debug view',
        ),
        DeclareLaunchArgument(
            'vio_method',
            default_value='none',
            description=(
                'Fuente de odometría para mover el grid: '
                'none | imu_only | visual_only | full'
            ),
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=depth_to_matrix_cfg,
            description='Path to depth_to_matrix params YAML',
        ),

        # Nodes — in pipeline order
        *realsense_actions,
        # Madgwick arranca junto con la cámara — necesita acumular
        # algunos segundos de datos antes de que la orientación converja
        imu_filter,
        # rgbd_odometry arranca 5s después para que el RealSense publique
        # TFs estables y Madgwick haya convergido.
        # Solo si vio_method es "visual_only" o "full".
        GroupAction(
            condition=IfCondition(use_visual_odom),
            actions=[TimerAction(period=5.0, actions=[rgbd_odometry])],
        ),
        local_mapper,
        depth_to_matrix,
        haptic_grid,
        hw_manager,
        viz,
        rviz,
    ])
