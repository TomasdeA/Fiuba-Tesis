#!/usr/bin/env python3
"""
test_depth_projection_visual.py

Publica una imagen de profundidad sintética + CameraInfo para validar
visualmente el DepthProjector en RViz.

Escena sintética:
  - Fondo: pared plana a 2.0 m.
  - Objeto: bloque centrado de 100×100 px a 1.0 m.
  - Zona inválida: franja izquierda con depth=0 (sin retorno).

Uso:
  # Terminal 1: levantar el nodo
  ros2 run depth_obstacle_filter depth_obstacle_filter_node

  # Terminal 2: publicar la escena sintética
  ros2 run depth_obstacle_filter test_depth_projection_visual.py

  # Terminal 3: abrir RViz
  rviz2 -d src/output_viewer/rviz/nav.rviz

En RViz se debe observar:
  - Un plano grande (la pared) a z=2.0 m
  - Un cuadrado más cercano (el objeto) a z=1.0 m
  - Un hueco a la izquierda (zona sin puntos)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
import struct


class SyntheticDepthPublisher(Node):
    def __init__(self):
        super().__init__('synthetic_depth_publisher')

        # Mismos topics genéricos que usa el nodo (sin remapping → directo)
        self.depth_pub = self.create_publisher(Image, 'camera/camera/depth/image_rect_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, 'camera/camera/depth/camera_info', 10)

        # Publicar a 5 Hz (suficiente para visualización)
        self.timer = self.create_timer(0.2, self.publish_frame)
        self.frame_id = 'camera_depth_optical_frame'

        # Resolución de la imagen sintética
        self.width = 640
        self.height = 480

        # Intrínsecos
        self.fx = 383.0
        self.fy = 383.0
        self.cx = 320.0
        self.cy = 240.0

        # Construir la imagen de profundidad sintética (una sola vez)
        self.depth_image = self._build_scene()

        self.get_logger().info(
            f'Publicando escena sintética {self.width}x{self.height} a 5 Hz. '
            f'Abrir RViz para visualizar.')

    def _build_scene(self) -> np.ndarray:
        """Construye la imagen de profundidad sintética (16-bit, mm)."""
        img = np.full((self.height, self.width), 2000, dtype=np.uint16)  # pared a 2 m

        # Objeto centrado: bloque de 100×100 px a 1.0 m
        cy, cx = self.height // 2, self.width // 2
        img[cy - 50:cy + 50, cx - 50:cx + 50] = 1000  # 1.0 m

        # Zona inválida: franja izquierda (depth=0)
        img[:, :40] = 0

        return img

    def publish_frame(self):
        now = self.get_clock().now().to_msg()

        # CameraInfo
        info = CameraInfo()
        info.header.stamp = now
        info.header.frame_id = self.frame_id
        info.width = self.width
        info.height = self.height
        info.k = [
            self.fx, 0.0,     self.cx,
            0.0,     self.fy, self.cy,
            0.0,     0.0,     1.0
        ]
        self.info_pub.publish(info)

        # Image (16UC1)
        img_msg = Image()
        img_msg.header.stamp = now
        img_msg.header.frame_id = self.frame_id
        img_msg.width = self.width
        img_msg.height = self.height
        img_msg.encoding = '16UC1'
        img_msg.is_bigendian = False
        img_msg.step = self.width * 2  # 2 bytes por píxel
        img_msg.data = self.depth_image.tobytes()
        self.depth_pub.publish(img_msg)


def main():
    rclpy.init()
    node = SyntheticDepthPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
