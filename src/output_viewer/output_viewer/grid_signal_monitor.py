import math

import rclpy
from rclpy.node import Node
from custom_interfaces.msg import DepthGrid
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2


class GridSignalMonitor(Node):
    def __init__(self):
        super().__init__('grid_signal_monitor')

        self.declare_parameter('grid_topic', '/perception/depth_grid')
        self.declare_parameter('cloud_topic', '/depth_obstacle_filter/obstacle_cloud')
        self.declare_parameter('depth_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('period_s', 1.0)
        self.declare_parameter('stale_timeout_s', 0.5)
        self.declare_parameter('startup_grace_s', 3.0)
        self.declare_parameter('min_depth_hz', 15.0)
        self.declare_parameter('min_grid_hz', 8.0)
        self.declare_parameter('min_cloud_hz', 8.0)

        self.grid_topic = self.get_parameter('grid_topic').value
        self.cloud_topic = self.get_parameter('cloud_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.period_s = float(self.get_parameter('period_s').value)
        self.stale_timeout_s = float(self.get_parameter('stale_timeout_s').value)
        self.startup_grace_s = float(self.get_parameter('startup_grace_s').value)
        self.min_depth_hz = float(self.get_parameter('min_depth_hz').value)
        self.min_grid_hz = float(self.get_parameter('min_grid_hz').value)
        self.min_cloud_hz = float(self.get_parameter('min_cloud_hz').value)

        self.depth_count = 0
        self.grid_count = 0
        self.cloud_count = 0
        self.empty_count = 0
        self.empty_streak = 0
        self.max_empty_streak = 0
        self.last_depth_time = None
        self.last_grid_time = None
        self.last_cloud_time = None
        self.last_depth_stamp = None
        self.last_nonempty = 0
        self.last_min_m = math.nan
        self.last_points = 0
        self.start_time = self.get_clock().now()

        self.create_subscription(Image, self.depth_topic, self.on_depth, 10)
        self.create_subscription(DepthGrid, self.grid_topic, self.on_grid, 10)
        self.create_subscription(PointCloud2, self.cloud_topic, self.on_cloud, 10)
        self.create_timer(self.period_s, self.on_timer)

        self.get_logger().info(
            f'GridSignalMonitor: depth={self.depth_topic} grid={self.grid_topic} '
            f'cloud={self.cloud_topic}')

    def on_depth(self, msg: Image):
        self.depth_count += 1
        self.last_depth_time = self.get_clock().now()
        self.last_depth_stamp = msg.header.stamp

    def on_cloud(self, msg: PointCloud2):
        self.cloud_count += 1
        self.last_cloud_time = self.get_clock().now()
        self.last_points = int(msg.width) * int(msg.height)

    def on_grid(self, msg: DepthGrid):
        self.grid_count += 1
        self.last_grid_time = self.get_clock().now()

        nonempty = 0
        min_m = math.inf
        for cell in msg.cells:
            if int(cell.count) <= 0:
                continue
            nonempty += 1
            if math.isfinite(float(cell.min_m)):
                min_m = min(min_m, float(cell.min_m))

        self.last_nonempty = nonempty
        self.last_min_m = min_m if math.isfinite(min_m) else math.nan
        if nonempty == 0:
            self.empty_count += 1
            self.empty_streak += 1
            self.max_empty_streak = max(self.max_empty_streak, self.empty_streak)
        else:
            self.empty_streak = 0

    def age_s(self, stamp):
        if stamp is None:
            return math.inf
        return (self.get_clock().now() - stamp).nanoseconds / 1e9

    def on_timer(self):
        depth_hz = self.depth_count / self.period_s
        grid_hz = self.grid_count / self.period_s
        cloud_hz = self.cloud_count / self.period_s
        depth_age = self.age_s(self.last_depth_time)
        grid_age = self.age_s(self.last_grid_time)
        cloud_age = self.age_s(self.last_cloud_time)

        self.get_logger().info(
            'signal depth_hz={:.1f} cloud_hz={:.1f} grid_hz={:.1f} '
            'depth_age={:.2f}s cloud_age={:.2f}s grid_age={:.2f}s '
            'cloud_pts={} cells={} min={:.2f} empty={} max_empty_streak={}'.format(
                depth_hz, cloud_hz, grid_hz, depth_age, cloud_age, grid_age,
                self.last_points, self.last_nonempty, self.last_min_m,
                self.empty_count, self.max_empty_streak))

        if self.age_s(self.start_time) < self.startup_grace_s:
            pass
        elif depth_age > self.stale_timeout_s:
            self.get_logger().warn(
                'depth raw stale: age={:.2f}s, el problema está en cámara/driver/input'.format(
                    depth_age))
        elif cloud_age > self.stale_timeout_s:
            self.get_logger().warn(
                'obstacle_cloud stale: age={:.2f}s, el problema está antes del encoder'.format(
                    cloud_age))
        elif grid_age > self.stale_timeout_s:
            self.get_logger().warn(
                'depth_grid stale: age={:.2f}s, no llega señal al hardware'.format(
                    grid_age))
        elif depth_hz < self.min_depth_hz:
            self.get_logger().warn(
                'depth raw lento: {:.1f}Hz < {:.1f}Hz, revisar RealSense/input'.format(
                    depth_hz, self.min_depth_hz))
        elif cloud_hz < self.min_cloud_hz:
            self.get_logger().warn(
                'obstacle_cloud lento: {:.1f}Hz < {:.1f}Hz, cuello antes del encoder'.format(
                    cloud_hz, self.min_cloud_hz))
        elif grid_hz < self.min_grid_hz:
            self.get_logger().warn(
                'depth_grid lento: {:.1f}Hz < {:.1f}Hz, revisar encoder/hardware'.format(
                    grid_hz, self.min_grid_hz))
        elif self.empty_streak > 0:
            self.get_logger().warn(
                'depth_grid vacío: streak={}, el corte está en filtered cloud/grid'.format(
                self.empty_streak))

        self.depth_count = 0
        self.grid_count = 0
        self.cloud_count = 0
        self.empty_count = 0
        self.max_empty_streak = self.empty_streak


def main(args=None):
    rclpy.init(args=args)
    node = GridSignalMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
