#!/usr/bin/env python3

from custom_interfaces.msg import PipelineMode

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy


_MODE_NAME_TO_VALUE = {
    'raw': PipelineMode.MODE_RAW,
    'filtered': PipelineMode.MODE_FILTERED,
    'full': PipelineMode.MODE_FULL,
}


class PipelineModeBridge(Node):

    def __init__(self):
        super().__init__('pipeline_mode_bridge')

        default_mode_name = self.declare_parameter('default_mode', 'raw').value
        default_mode_name = str(default_mode_name).strip().lower()
        if default_mode_name not in _MODE_NAME_TO_VALUE:
            self.get_logger().warn(
                f'default_mode="{default_mode_name}" invalido; usando raw'
            )
            default_mode_name = 'raw'

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.selected_topic = str(
            self.declare_parameter(
                'selected_topic',
                '/pipeline/selected_mode',
            ).value
        )
        self.command_topic = str(
            self.declare_parameter(
                'command_topic',
                '/pipeline/selected_mode_cmd',
            ).value
        )
        self.pub = self.create_publisher(PipelineMode, self.selected_topic, qos)
        self.sub = self.create_subscription(
            PipelineMode,
            self.command_topic,
            self.on_command,
            10,
        )

        self.current_mode = _MODE_NAME_TO_VALUE[default_mode_name]
        self.publish_mode()
        self.get_logger().info(
            f'PipelineModeBridge listo. selected_topic={self.selected_topic} '
            f'command_topic={self.command_topic} modo_inicial="{default_mode_name}"'
        )

    def on_command(self, msg: PipelineMode):
        if int(msg.mode) not in (
            PipelineMode.MODE_RAW,
            PipelineMode.MODE_FILTERED,
            PipelineMode.MODE_FULL,
        ):
            self.get_logger().warn(f'Modo de pipeline invalido recibido: {int(msg.mode)}')
            return

        if int(msg.mode) == int(self.current_mode):
            return

        self.current_mode = int(msg.mode)
        self.publish_mode()

    def publish_mode(self):
        msg = PipelineMode()
        msg.mode = int(self.current_mode)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PipelineModeBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
