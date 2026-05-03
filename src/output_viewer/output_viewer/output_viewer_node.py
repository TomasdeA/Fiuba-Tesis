#!/usr/bin/env python3

from custom_interfaces.msg import DepthGrid

import matplotlib.pyplot as plt
import numpy as np

import rclpy
from rclpy.node import Node


def extract_distance_and_count(msg: DepthGrid):
    rows = int(msg.rows)
    cols = int(msg.cols)

    d = np.array([float(c.mean_m) for c in msg.cells], dtype=np.float32)
    cnt = np.array([int(c.count) for c in msg.cells], dtype=np.int32)

    if d.size != rows * cols:
        raise RuntimeError(f'Tamaño inconsistente: rows*cols={rows*cols} pero llegó {d.size}')

    return d.reshape((rows, cols)), cnt.reshape((rows, cols))


def distance_to_intensity(d_m: np.ndarray, z_min: float, z_max: float) -> np.ndarray:
    d = np.clip(d_m, z_min, z_max)
    intensity = (z_max - d) / (z_max - z_min) * 100.0
    return intensity.astype(np.float32)


class DepthGridHeatmapNode(Node):

    def __init__(self):
        super().__init__('depth_grid_heatmap')

        self.declare_parameter('topic', '/perception/depth_grid')
        self.declare_parameter('z_min', 0.6)
        self.declare_parameter('z_max', 4.0)
        self.declare_parameter('show_values', True)
        self.declare_parameter('invalid_text', '--')
        self.declare_parameter('refresh_hz', 20.0)

        self.topic = str(self.get_parameter('topic').value)
        self.z_min = float(self.get_parameter('z_min').value)
        self.z_max = float(self.get_parameter('z_max').value)
        self.show_values = bool(self.get_parameter('show_values').value)
        self.invalid_text = str(self.get_parameter('invalid_text').value)
        self.refresh_hz = float(self.get_parameter('refresh_hz').value)

        self.sub = self.create_subscription(DepthGrid, self.topic, self.cb, 10)

        # Estado compartido (actualizado por el callback)
        self.latest_m = None     # intensidad 0..100
        self.latest_cnt = None   # count
        self.latest_shape = None
        self._dirty = False

        # Matplotlib setup (sin pause en callback)
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.im = None
        self.cbar = None
        self.texts = []
        self._closing = False

        self.ax.set_title(f'Heatmap intensidades (rojo=cerca): {self.topic}')
        self.ax.set_xlabel('col')
        self.ax.set_ylabel('row')

        self.fig.canvas.mpl_connect('close_event', self._on_close)

        # Timer ROS para refrescar el plot
        period = 1.0 / max(self.refresh_hz, 1.0)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            f'Escuchando {self.topic} | z_min={self.z_min}m '
            f'z_max={self.z_max}m | refresh={self.refresh_hz}Hz'
        )

    def _on_close(self, _evt):
        # Usuario cerró la ventana: marcamos cierre y apagamos ROS
        self._closing = True
        try:
            rclpy.shutdown()
        except Exception:
            pass

    def _clear_texts(self):
        for t in self.texts:
            try:
                t.remove()
            except Exception:
                pass
        self.texts = []

    def _setup_axes_ticks(self, rows: int, cols: int):
        self.ax.set_xticks(np.arange(cols))
        self.ax.set_yticks(np.arange(rows))
        self.ax.set_xlim(-0.5, cols - 0.5)
        self.ax.set_ylim(rows - 0.5, -0.5)  # origin upper
        self.ax.grid(False)

    def cb(self, msg: DepthGrid):
        # Solo computa y guarda; NO dibuja acá
        try:
            d_m, cnt = extract_distance_and_count(msg)
        except Exception as e:
            self.get_logger().error(f'No pude parsear DepthGrid: {e}')
            return

        # celdas sin puntos -> lejos
        d_m = np.where(cnt > 0, d_m, self.z_max)
        d_m = np.nan_to_num(d_m, nan=self.z_max, posinf=self.z_max, neginf=self.z_max)

        m = distance_to_intensity(d_m, self.z_min, self.z_max)

        self.latest_m = m
        self.latest_cnt = cnt
        self.latest_shape = m.shape
        self._dirty = True

    def on_timer(self):
        if self._closing:
            return
        if self.latest_m is None or not self._dirty:
            return

        m = self.latest_m
        cnt = self.latest_cnt
        rows, cols = m.shape

        try:
            if self.im is None or self.im.get_array().shape != m.shape:
                self.ax.clear()
                self.ax.set_title(f'Heatmap intensidades (rojo=cerca): {self.topic}')
                self.ax.set_xlabel('col')
                self.ax.set_ylabel('row')

                self.im = self.ax.imshow(
                    m,
                    vmin=0.0,
                    vmax=100.0,
                    cmap='RdBu_r',
                    interpolation='nearest',
                    aspect='equal',
                    origin='upper',
                )

                self._setup_axes_ticks(rows, cols)

                if self.cbar is not None:
                    try:
                        self.cbar.remove()
                    except Exception:
                        pass
                self.cbar = self.fig.colorbar(self.im, ax=self.ax, shrink=0.9)
            else:
                self.im.set_data(m)

            if self.show_values:
                self._clear_texts()
                for r in range(rows):
                    for c in range(cols):
                        if cnt[r, c] <= 0:
                            txt = self.invalid_text
                        else:
                            v = m[r, c]
                            txt = self.invalid_text if not np.isfinite(v) else f'{int(round(v))}'
                        self.texts.append(
                            self.ax.text(c, r, txt, ha='center', va='center', fontsize=9)
                        )
            else:
                if self.texts:
                    self._clear_texts()

            # draw sin pause (evita lios con Ctrl+C en callbacks)
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()

            self._dirty = False

        except Exception as e:
            # Si está cerrándose la ventana, evitamos spam de errores
            if not self._closing:
                self.get_logger().warn(f'Error dibujando heatmap: {e}')


def main():
    rclpy.init()
    node = DepthGridHeatmapNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cierre prolijo sin trazar Tkinter
        node._closing = True
        try:
            if plt.fignum_exists(node.fig.number):
                plt.close(node.fig)
        except Exception:
            pass

        try:
            node.destroy_node()
        except Exception:
            pass

        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
