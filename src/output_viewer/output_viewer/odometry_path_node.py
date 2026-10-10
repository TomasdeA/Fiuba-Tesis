"""
Nodo de diagnóstico: acumula la posición de nav_odom y publica nav_msgs/Path.

Útil para verificar la calidad de la odometría independientemente del mapeo.

Publicaciones:
  /debug/odom_path  (nav_msgs/Path, frame_id=odom)

Uso rápido:
  ros2 run output_viewer odometry_path
  rviz2 → Add → By topic → /debug/odom_path → Path
"""

import copy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from output_viewer.odometry_conventions import (
    rtabmap_position,
    rtabmap_yaw_quaternion,
)
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32


class OdometryPathNode(Node):

    def __init__(self):
        super().__init__('odometry_path')

        self.declare_parameter('max_poses', 10000)
        self.declare_parameter('odom_source', 'nav_odom')
        self.max_poses = self.get_parameter('max_poses').value
        self._odom_source = self.get_parameter(
            'odom_source'
        ).get_parameter_value().string_value
        if self._odom_source not in ('nav_odom', 'rtabmap_odom'):
            self.get_logger().warning(
                f"odom_source='{self._odom_source}' inválido; usando nav_odom"
            )
            self._odom_source = 'nav_odom'

        self._floor_y: float = 0.0          # altura del piso en frame odom (de camera_height)
        self._floor_y_received: bool = False  # no emitir poses hasta tener la primera lectura

        self._path = Path()
        self._path.header.frame_id = 'odom'

        qos_pub = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._pub = self.create_publisher(Path, '/debug/odom_path', qos_pub)
        self._sub = self.create_subscription(
            Odometry, 'nav_odom', self._on_odom, 10
        )
        qos_latch = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._height_sub = self.create_subscription(
            Float32, '/depth_obstacle_filter/camera_height', self._on_camera_height, qos_latch
        )

        self.get_logger().info(
            'odometry_path: nav_odom → /debug/odom_path '
            f'(odom_source={self._odom_source})'
        )

    def _on_camera_height(self, msg: Float32):
        self._floor_y = float(msg.data)
        self._floor_y_received = True

    def _on_odom(self, msg: Odometry):
        if not self._floor_y_received:
            return  # esperar a tener h_piso para no generar la línea vertical desde y=0

        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = copy.deepcopy(msg.pose.pose)

        if self._odom_source == 'rtabmap_odom':
            # RTAB-Map usa el plano ROS XY (X adelante, Y izquierda). El mapper
            # usa el plano óptico/interno XZ (Z adelante, X derecha).
            ros_x = msg.pose.pose.position.x
            ros_y = msg.pose.pose.position.y
            internal_x, internal_z = rtabmap_position(ros_x, ros_y)
            pose.pose.position.x = internal_x
            pose.pose.position.z = internal_z

            qw, qx, qy, qz = rtabmap_yaw_quaternion(
                msg.pose.pose.orientation
            )
            pose.pose.orientation.w = qw
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz

        # Proyectar la trayectoria al piso estimado por GroundEstimator.
        pose.pose.position.y = self._floor_y

        self._path.poses.append(pose)

        if len(self._path.poses) > self.max_poses:
            self._path.poses = self._path.poses[-self.max_poses:]

        self._path.header.stamp = msg.header.stamp
        self._pub.publish(self._path)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryPathNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
