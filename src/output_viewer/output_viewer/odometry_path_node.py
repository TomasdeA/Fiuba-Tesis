"""
Nodo de diagnóstico: acumula la posición XZ de nav_odom y publica nav_msgs/Path.
Útil para verificar la calidad de la odometría independientemente del mapeo.

Publicaciones:
  /debug/odom_path  (nav_msgs/Path, frame_id=odom)

Uso rápido:
  ros2 run output_viewer odometry_path
  rviz2 → Add → By topic → /debug/odom_path → Path
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32


class OdometryPathNode(Node):
    def __init__(self):
        super().__init__('odometry_path')

        self.declare_parameter('max_poses', 10000)
        self.max_poses = self.get_parameter('max_poses').value
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

        self.get_logger().info('odometry_path: suscrito a nav_odom → /debug/odom_path')

    def _on_camera_height(self, msg: Float32):
        self._floor_y = float(msg.data)
        self._floor_y_received = True

    def _on_odom(self, msg: Odometry):
        if not self._floor_y_received:
            return  # esperar a tener h_piso para no generar la línea vertical desde y=0

        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        pose.pose.position.y = self._floor_y  # proyectar al piso estimado por GroundEstimator

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
