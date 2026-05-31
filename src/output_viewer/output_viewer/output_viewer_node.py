#!/usr/bin/env python3

from custom_interfaces.msg import DepthGrid
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import SetParameters

import math
import time

import numpy as np

import rclpy
from rclpy.node import Node


BG = (245, 245, 245)
FG = (25, 25, 25)
GRID = (215, 215, 215)
INVALID_BG = (238, 238, 238)
INVALID_FG = (90, 90, 90)
pygame = None


def extract_distance_and_count(msg: DepthGrid):
    rows = int(msg.rows)
    cols = int(msg.cols)

    d = np.array([float(c.min_m) for c in msg.cells], dtype=np.float32)
    cnt = np.array([int(c.count) for c in msg.cells], dtype=np.int32)

    if d.size != rows * cols:
        raise RuntimeError(
            f'Tamaño inconsistente: rows*cols={rows*cols} '
            f'pero llegó {d.size}'
        )

    return d.reshape((rows, cols)), cnt.reshape((rows, cols))


def distance_to_intensity(
    d_m: np.ndarray,
    z_min: float,
    z_max: float,
) -> np.ndarray:
    d = np.clip(d_m, z_min, z_max)
    intensity = (z_max - d) / (z_max - z_min) * 100.0
    return intensity.astype(np.float32)


def intensity_to_rgb(m: np.ndarray) -> np.ndarray:
    """Colormap tipo RdBu_r."""
    x = np.clip(m.astype(np.float32) / 100.0, 0.0, 1.0)

    blue = np.array([5, 48, 97], dtype=np.float32)
    light_blue = np.array([146, 197, 222], dtype=np.float32)
    white = np.array([247, 247, 247], dtype=np.float32)
    light_red = np.array([244, 165, 130], dtype=np.float32)
    red = np.array([178, 24, 43], dtype=np.float32)

    stops = np.stack((blue, light_blue, white, light_red, red), axis=0)
    scaled = x * (len(stops) - 1)
    idx = np.floor(scaled).astype(np.int32)
    idx = np.clip(idx, 0, len(stops) - 2)
    t = (scaled - idx)[..., None]

    rgb = stops[idx] * (1.0 - t) + stops[idx + 1] * t
    return rgb.astype(np.uint8)


def display_matrix(data: np.ndarray, flip_rows: bool) -> np.ndarray:
    if not flip_rows:
        return data
    return np.flipud(data)


def display_row(rows: int, row: int, flip_rows: bool) -> int:
    if not flip_rows:
        return row
    return rows - 1 - row


class DepthGridHeatmapNode(Node):

    def __init__(self):
        super().__init__('depth_grid_heatmap')

        self.declare_parameter('topic', '/perception/depth_grid')
        self.declare_parameter('z_min', 0.6)
        self.declare_parameter('z_max', 4.0)
        self.declare_parameter('show_values', True)
        self.declare_parameter('invalid_text', '--')
        self.declare_parameter('refresh_hz', 20.0)
        self.declare_parameter('render_backend', 'pygame')
        self.declare_parameter('flip_rows_for_display', False)
        self.declare_parameter('pygame_view_mode', 'curved')
        self.declare_parameter('aperture_control_enabled', True)
        self.declare_parameter('encoder_node_name', '/obstacle_grid_encoder')
        self.declare_parameter('h_aperture_deg', 45.0)
        self.declare_parameter('h_aperture_min_deg', 15.0)
        self.declare_parameter('h_aperture_max_deg', 70.0)

        self.topic = str(self.get_parameter('topic').value)
        self.z_min = float(self.get_parameter('z_min').value)
        self.z_max = float(self.get_parameter('z_max').value)
        self.show_values = bool(self.get_parameter('show_values').value)
        self.invalid_text = str(self.get_parameter('invalid_text').value)
        self.refresh_hz = float(self.get_parameter('refresh_hz').value)
        self.flip_rows_for_display = bool(
            self.get_parameter('flip_rows_for_display').value
        )
        self.pygame_view_mode = str(
            self.get_parameter('pygame_view_mode').value
        ).lower()
        if self.pygame_view_mode not in ('curved', 'grid'):
            self.get_logger().warn(
                f'pygame_view_mode="{self.pygame_view_mode}" invalido; '
                'usando curved'
            )
            self.pygame_view_mode = 'curved'
        self.aperture_control_enabled = bool(
            self.get_parameter('aperture_control_enabled').value
        )
        self.encoder_node_name = str(
            self.get_parameter('encoder_node_name').value
        )
        self.h_aperture_deg = float(
            self.get_parameter('h_aperture_deg').value
        )
        self.h_aperture_min_deg = float(
            self.get_parameter('h_aperture_min_deg').value
        )
        self.h_aperture_max_deg = float(
            self.get_parameter('h_aperture_max_deg').value
        )
        self.h_aperture_deg = float(np.clip(
            self.h_aperture_deg,
            self.h_aperture_min_deg,
            self.h_aperture_max_deg,
        ))
        self.render_backend = str(
            self.get_parameter('render_backend').value
        ).lower()
        if self.render_backend not in ('pygame', 'matplotlib'):
            self.get_logger().warn(
                f'render_backend="{self.render_backend}" invalido; '
                'usando pygame'
            )
            self.render_backend = 'pygame'

        self.sub = self.create_subscription(DepthGrid, self.topic, self.cb, 10)

        # Estado compartido (actualizado por el callback)
        self.latest_m = None     # intensidad 0..100
        self.latest_cnt = None   # count
        self.latest_shape = None
        self._dirty = False
        self._dragging_aperture = False
        self._last_aperture_send_s = 0.0
        self._aperture_pending = False
        self._aperture_client = None
        if self.aperture_control_enabled:
            service_name = f'{self.encoder_node_name}/set_parameters'
            self._aperture_client = self.create_client(
                SetParameters,
                service_name,
            )
            self._aperture_pending = True

        self._closing = False
        if self.render_backend == 'matplotlib':
            self._setup_matplotlib()
        else:
            self._setup_pygame()

        # Timer ROS para refrescar el plot
        period = 1.0 / max(self.refresh_hz, 1.0)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            f'Escuchando {self.topic} | z_min={self.z_min}m '
            f'z_max={self.z_max}m | refresh={self.refresh_hz}Hz '
            f'| backend={self.render_backend} '
            f'| flip_rows={self.flip_rows_for_display} '
            f'| pygame_view={self.pygame_view_mode} '
            f'| aperture={self.h_aperture_deg:.1f}deg'
        )

    def _setup_matplotlib(self):
        import matplotlib.pyplot as plt

        self.plt = plt
        self.plt.ion()
        self.fig, self.ax = self.plt.subplots()
        self.im = None
        self.cbar = None
        self.texts = []

        self.ax.set_title(f'Heatmap intensidades (rojo=cerca): {self.topic}')
        self.ax.set_xlabel('col')
        self.ax.set_ylabel('row')

        self.fig.canvas.mpl_connect('close_event', self._on_close)

    def _setup_pygame(self):
        global pygame

        import pygame as pygame_module

        pygame = pygame_module
        pygame_module.init()
        pygame_module.font.init()
        self.screen = pygame.display.set_mode((900, 700), pygame.RESIZABLE)
        pygame.display.set_caption(
            f'Heatmap intensidades (rojo=cerca): {self.topic}'
        )
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont(None, 18)
        self.value_font = pygame.font.SysFont(None, 20)
        self.small_font = pygame.font.SysFont(None, 16)
        self._draw_waiting()

    def _on_close(self, _evt=None):
        # Usuario cerró la ventana: marcamos cierre y apagamos ROS
        self._closing = True
        try:
            rclpy.shutdown()
        except Exception:
            pass

    def cb(self, msg: DepthGrid):
        # Solo computa y guarda; NO dibuja acá
        try:
            d_m, cnt = extract_distance_and_count(msg)
        except Exception as e:
            self.get_logger().error(f'No pude parsear DepthGrid: {e}')
            return

        # celdas sin puntos -> lejos
        d_m = np.where(cnt > 0, d_m, self.z_max)
        d_m = np.nan_to_num(
            d_m,
            nan=self.z_max,
            posinf=self.z_max,
            neginf=self.z_max,
        )

        m = distance_to_intensity(d_m, self.z_min, self.z_max)

        self.latest_m = m
        self.latest_cnt = cnt
        self.latest_shape = m.shape
        self._dirty = True

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
        if self.flip_rows_for_display:
            self.ax.set_yticklabels([str(rows - 1 - r) for r in range(rows)])
        self.ax.set_xlim(-0.5, cols - 0.5)
        self.ax.set_ylim(rows - 0.5, -0.5)  # pantalla: y crece hacia abajo
        self.ax.grid(False)

    def _slider_rect(self):
        width, _height = self.screen.get_size()
        return pygame.Rect(190, 46, max(220, width - 390), 10)

    def _aperture_from_x(self, x: int) -> float:
        rect = self._slider_rect()
        t = (x - rect.left) / max(1, rect.width)
        t = float(np.clip(t, 0.0, 1.0))
        return (
            self.h_aperture_min_deg +
            t * (self.h_aperture_max_deg - self.h_aperture_min_deg)
        )

    def _set_local_aperture(self, value: float):
        self.h_aperture_deg = float(np.clip(
            value,
            self.h_aperture_min_deg,
            self.h_aperture_max_deg,
        ))
        self._dirty = True

    def _send_aperture(self, force=False):
        if self._aperture_client is None:
            return

        now = time.monotonic()
        if not force and now - self._last_aperture_send_s < 0.15:
            return
        self._last_aperture_send_s = now

        param = Parameter()
        param.name = 'h_aperture_deg'
        param.value = ParameterValue()
        param.value.type = ParameterType.PARAMETER_DOUBLE
        param.value.double_value = float(self.h_aperture_deg)

        req = SetParameters.Request()
        req.parameters = [param]
        if self._aperture_client.service_is_ready():
            future = self._aperture_client.call_async(req)
            future.add_done_callback(self._on_aperture_response)
            self._aperture_pending = False
        else:
            self._aperture_pending = True

    def _on_aperture_response(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(
                f'No pude actualizar h_aperture_deg en '
                f'{self.encoder_node_name}: {exc}'
            )
            self._aperture_pending = True
            return

        if not response.results:
            self.get_logger().warn(
                f'{self.encoder_node_name}/set_parameters no devolvio resultado'
            )
            self._aperture_pending = True
            return

        result = response.results[0]
        if not result.successful:
            self.get_logger().warn(
                f'{self.encoder_node_name} rechazo h_aperture_deg: '
                f'{result.reason}'
            )
            self._aperture_pending = True

    def _handle_aperture_event(self, event):
        if not self.aperture_control_enabled:
            return False

        rect = self._slider_rect().inflate(18, 22)
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if rect.collidepoint(event.pos):
                self._dragging_aperture = True
                self._set_local_aperture(self._aperture_from_x(event.pos[0]))
                self._send_aperture(force=True)
                return True

        if event.type == pygame.MOUSEMOTION and self._dragging_aperture:
            self._set_local_aperture(self._aperture_from_x(event.pos[0]))
            self._send_aperture()
            return True

        if event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            if self._dragging_aperture:
                self._dragging_aperture = False
                self._set_local_aperture(self._aperture_from_x(event.pos[0]))
                self._send_aperture(force=True)
                return True

        return False

    def _handle_events(self):
        for event in pygame.event.get():
            if self._handle_aperture_event(event):
                continue
            if event.type == pygame.QUIT:
                self._on_close()
            elif (
                event.type == pygame.KEYDOWN
                and event.key in (pygame.K_ESCAPE, pygame.K_q)
            ):
                self._on_close()
            elif (
                event.type == pygame.VIDEORESIZE
                and self.latest_m is not None
            ):
                self._dirty = True

    def _draw_waiting(self):
        self.screen.fill(BG)
        self._draw_text(
            f'Heatmap intensidades (rojo=cerca): {self.topic}',
            (24, 14),
            self.font,
            FG,
        )
        self._draw_aperture_slider()
        self._draw_text('Esperando DepthGrid...', (24, 76), self.font, FG)
        pygame.display.flip()

    def _draw_text(
        self,
        text: str,
        pos,
        font=None,
        color=FG,
        center=False,
    ):
        surf = (font or self.font).render(text, True, color)
        rect = surf.get_rect()
        if center:
            rect.center = pos
        else:
            rect.topleft = pos
        self.screen.blit(surf, rect)
        return rect

    def _draw_aperture_slider(self):
        if not self.aperture_control_enabled:
            return

        rect = self._slider_rect()
        label = f'apertura horizontal ±{self.h_aperture_deg:.0f}°'
        self._draw_text(label, (24, 38), self.font, FG)

        pygame.draw.line(
            self.screen,
            (170, 170, 170),
            rect.midleft,
            rect.midright,
            4,
        )
        t = (
            (self.h_aperture_deg - self.h_aperture_min_deg) /
            max(1e-3, self.h_aperture_max_deg - self.h_aperture_min_deg)
        )
        knob_x = rect.left + int(round(t * rect.width))
        pygame.draw.circle(self.screen, (35, 35, 35), (knob_x, rect.centery), 8)

        self._draw_text(
            f'{self.h_aperture_min_deg:.0f}°',
            (rect.left, rect.bottom + 8),
            self.small_font,
            FG,
            center=True,
        )
        self._draw_text(
            f'{self.h_aperture_max_deg:.0f}°',
            (rect.right, rect.bottom + 8),
            self.small_font,
            FG,
            center=True,
        )

    def _cell_rect(
        self,
        plot_rect,
        rows: int,
        cols: int,
        row: int,
        col: int,
    ):
        left = plot_rect.left + round(col * plot_rect.width / cols)
        right = plot_rect.left + round((col + 1) * plot_rect.width / cols)
        top = plot_rect.top + round(row * plot_rect.height / rows)
        bottom = plot_rect.top + round((row + 1) * plot_rect.height / rows)
        return pygame.Rect(
            left,
            top,
            max(1, right - left),
            max(1, bottom - top),
        )

    def _layout(self, rows: int, cols: int):
        width, height = self.screen.get_size()
        margin = 24
        title_h = 72
        tick_left = 42
        tick_bottom = 32
        cbar_w = 28
        cbar_gap = 18
        right_pad = cbar_w + cbar_gap + 48

        max_w = max(40, width - margin * 2 - tick_left - right_pad)
        max_h = max(40, height - margin * 2 - title_h - tick_bottom)
        scale = max(1, min(max_w // max(cols, 1), max_h // max(rows, 1)))

        plot_w = max(cols, cols * scale)
        plot_h = max(rows, rows * scale)
        left = margin + tick_left + max(0, (max_w - plot_w) // 2)
        top = margin + title_h + max(0, (max_h - plot_h) // 2)
        plot_rect = pygame.Rect(left, top, plot_w, plot_h)
        cbar_rect = pygame.Rect(
            plot_rect.right + cbar_gap,
            plot_rect.top,
            cbar_w,
            plot_rect.height,
        )
        return plot_rect, cbar_rect

    def _angle_edges(self, min_deg: float, max_deg: float, count: int):
        return np.deg2rad(np.linspace(min_deg, max_deg, count + 1))

    def _project_curved(self, rect, h_angle: float, row_pos: float):
        h_min = math.radians(-self.h_aperture_deg)
        h_max = math.radians(self.h_aperture_deg)

        max_abs_h = max(abs(h_min), abs(h_max), 1e-3)
        x_norm = math.sin(h_angle) / math.sin(max_abs_h)
        x = rect.centerx + x_norm * rect.width * 0.5

        curve_norm = (1.0 - math.cos(h_angle)) / max(
            1.0 - math.cos(max_abs_h),
            1e-3,
        )
        y = rect.top + row_pos * rect.height + curve_norm * 34.0
        return int(round(x)), int(round(y))

    def _draw_heatmap_curved(self, m: np.ndarray, cnt: np.ndarray):
        rows, cols = m.shape
        self.screen.fill(BG)
        self._draw_text(
            f'Heatmap angular: {self.topic}',
            (24, 14),
            self.font,
            FG,
        )
        self._draw_aperture_slider()

        plot_rect, cbar_rect = self._layout(rows, cols)
        plot_rect = plot_rect.inflate(-8, -42)
        plot_rect.top += 14

        rgb = intensity_to_rgb(m)
        h_edges = self._angle_edges(
            -self.h_aperture_deg,
            self.h_aperture_deg,
            cols,
        )
        row_edges = np.linspace(0.0, 1.0, rows + 1)

        for r in range(rows):
            view_r = display_row(rows, r, self.flip_rows_for_display)
            for c in range(cols):
                color = INVALID_BG if cnt[r, c] <= 0 else tuple(rgb[r, c])
                points = [
                    self._project_curved(
                        plot_rect,
                        h_edges[c],
                        row_edges[view_r],
                    ),
                    self._project_curved(
                        plot_rect,
                        h_edges[c + 1],
                        row_edges[view_r],
                    ),
                    self._project_curved(
                        plot_rect,
                        h_edges[c + 1],
                        row_edges[view_r + 1],
                    ),
                    self._project_curved(
                        plot_rect,
                        h_edges[c],
                        row_edges[view_r + 1],
                    ),
                ]
                pygame.draw.polygon(self.screen, color, points)
                pygame.draw.lines(self.screen, GRID, True, points, 1)

                if self.show_values and cnt[r, c] > 0:
                    v = m[r, c]
                    txt = (
                        self.invalid_text
                        if not np.isfinite(v)
                        else f'{int(round(v))}'
                    )
                    cx = sum(p[0] for p in points) / 4.0
                    cy = sum(p[1] for p in points) / 4.0
                    self._draw_text(
                        txt,
                        (cx, cy),
                        self.value_font,
                        FG,
                        center=True,
                    )
                elif self.show_values:
                    cx = sum(p[0] for p in points) / 4.0
                    cy = sum(p[1] for p in points) / 4.0
                    self._draw_text(
                        self.invalid_text,
                        (cx, cy),
                        self.value_font,
                        INVALID_FG,
                        center=True,
                    )

        for c in range(cols + 1):
            angle_deg = -self.h_aperture_deg + (
                2.0 * self.h_aperture_deg
            ) * c / cols
            if c in (0, cols // 2, cols):
                p = self._project_curved(plot_rect, h_edges[c], row_edges[-1])
                self._draw_text(
                    f'{angle_deg:.0f}°',
                    (p[0], p[1] + 18),
                    self.small_font,
                    FG,
                    center=True,
                )

        gradient = np.linspace(
            100.0,
            0.0,
            max(cbar_rect.height, 1),
            dtype=np.float32,
        )[:, None]
        gradient_rgb = intensity_to_rgb(gradient)
        gradient_rgb = np.repeat(gradient_rgb, cbar_rect.width, axis=1)
        cbar_surf = pygame.surfarray.make_surface(
            np.transpose(gradient_rgb, (1, 0, 2))
        )
        self.screen.blit(cbar_surf, cbar_rect)
        pygame.draw.rect(self.screen, FG, cbar_rect, 1)
        self._draw_text(
            '100',
            (cbar_rect.right + 8, cbar_rect.top - 2),
            self.small_font,
            FG,
        )
        self._draw_text(
            '50',
            (cbar_rect.right + 8, cbar_rect.centery - 8),
            self.small_font,
            FG,
        )
        self._draw_text(
            '0',
            (cbar_rect.right + 8, cbar_rect.bottom - 14),
            self.small_font,
            FG,
        )

        pygame.display.flip()
        self.clock.tick(max(1.0, self.refresh_hz))

    def _draw_heatmap(self, m: np.ndarray, cnt: np.ndarray):
        if self.pygame_view_mode == 'curved':
            self._draw_heatmap_curved(m, cnt)
            return

        rows, cols = m.shape
        self.screen.fill(BG)

        self._draw_text(
            f'Heatmap intensidades (rojo=cerca): {self.topic}',
            (24, 14),
            self.font,
            FG,
        )
        self._draw_aperture_slider()

        plot_rect, cbar_rect = self._layout(rows, cols)
        rgb = intensity_to_rgb(m)
        rgb[cnt <= 0] = INVALID_BG
        rgb = display_matrix(rgb, self.flip_rows_for_display)

        surf = pygame.surfarray.make_surface(np.transpose(rgb, (1, 0, 2)))
        surf = pygame.transform.scale(surf, plot_rect.size)
        self.screen.blit(surf, plot_rect)

        for c in range(cols + 1):
            x = plot_rect.left + round(c * plot_rect.width / cols)
            pygame.draw.line(
                self.screen,
                GRID,
                (x, plot_rect.top),
                (x, plot_rect.bottom),
                1,
            )
        for r in range(rows + 1):
            y = plot_rect.top + round(r * plot_rect.height / rows)
            pygame.draw.line(
                self.screen,
                GRID,
                (plot_rect.left, y),
                (plot_rect.right, y),
                1,
            )

        for c in range(cols):
            x = plot_rect.left + round((c + 0.5) * plot_rect.width / cols)
            self._draw_text(
                str(c),
                (x, plot_rect.bottom + 12),
                self.small_font,
                FG,
                center=True,
            )
        for r in range(rows):
            view_r = display_row(rows, r, self.flip_rows_for_display)
            y = plot_rect.top + round(
                (view_r + 0.5) * plot_rect.height / rows
            )
            self._draw_text(
                str(r),
                (plot_rect.left - 16, y),
                self.small_font,
                FG,
                center=True,
            )

        self._draw_text(
            'col',
            (plot_rect.centerx, plot_rect.bottom + 28),
            self.small_font,
            FG,
            center=True,
        )
        self._draw_text(
            'row',
            (plot_rect.left - 34, plot_rect.centery),
            self.small_font,
            FG,
            center=True,
        )

        if self.show_values:
            for r in range(rows):
                for c in range(cols):
                    view_r = display_row(
                        rows,
                        r,
                        self.flip_rows_for_display,
                    )
                    cell = self._cell_rect(
                        plot_rect,
                        rows,
                        cols,
                        view_r,
                        c,
                    )
                    if cnt[r, c] <= 0:
                        txt = self.invalid_text
                        color = INVALID_FG
                    else:
                        v = m[r, c]
                        txt = (
                            self.invalid_text
                            if not np.isfinite(v)
                            else f'{int(round(v))}'
                        )
                        color = FG
                    if cell.width >= 18 and cell.height >= 14:
                        self._draw_text(
                            txt,
                            cell.center,
                            self.value_font,
                            color,
                            center=True,
                        )

        gradient = np.linspace(
            100.0,
            0.0,
            max(cbar_rect.height, 1),
            dtype=np.float32,
        )[:, None]
        gradient_rgb = intensity_to_rgb(gradient)
        gradient_rgb = np.repeat(gradient_rgb, cbar_rect.width, axis=1)
        cbar_surf = pygame.surfarray.make_surface(
            np.transpose(gradient_rgb, (1, 0, 2))
        )
        self.screen.blit(cbar_surf, cbar_rect)
        pygame.draw.rect(self.screen, FG, cbar_rect, 1)
        self._draw_text(
            '100',
            (cbar_rect.right + 8, cbar_rect.top - 2),
            self.small_font,
            FG,
        )
        self._draw_text(
            '50',
            (cbar_rect.right + 8, cbar_rect.centery - 8),
            self.small_font,
            FG,
        )
        self._draw_text(
            '0',
            (cbar_rect.right + 8, cbar_rect.bottom - 14),
            self.small_font,
            FG,
        )

        pygame.display.flip()
        self.clock.tick(max(1.0, self.refresh_hz))

    def _draw_heatmap_matplotlib(self, m: np.ndarray, cnt: np.ndarray):
        rows, cols = m.shape

        if self.im is None or self.im.get_array().shape != m.shape:
            self.ax.clear()
            self.ax.set_title(
                f'Heatmap intensidades (rojo=cerca): {self.topic}'
            )
            self.ax.set_xlabel('col')
            self.ax.set_ylabel('row')

            self.im = self.ax.imshow(
                display_matrix(m, self.flip_rows_for_display),
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
            self.im.set_data(display_matrix(m, self.flip_rows_for_display))

        if self.show_values:
            self._clear_texts()
            for r in range(rows):
                for c in range(cols):
                    view_r = display_row(
                        rows,
                        r,
                        self.flip_rows_for_display,
                    )
                    if cnt[r, c] <= 0:
                        txt = self.invalid_text
                    else:
                        v = m[r, c]
                        txt = (
                            self.invalid_text
                            if not np.isfinite(v)
                            else f'{int(round(v))}'
                        )
                    self.texts.append(
                        self.ax.text(
                            c,
                            view_r,
                            txt,
                            ha='center',
                            va='center',
                            fontsize=9,
                        )
                    )
        elif self.texts:
            self._clear_texts()

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def on_timer(self):
        if self.render_backend == 'pygame':
            self._handle_events()
            if self._aperture_pending:
                self._send_aperture(force=True)
        if self._closing:
            return
        if self.latest_m is None or not self._dirty:
            return

        m = self.latest_m
        cnt = self.latest_cnt

        try:
            if self.render_backend == 'matplotlib':
                self._draw_heatmap_matplotlib(m, cnt)
            else:
                self._draw_heatmap(m, cnt)
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
        node._closing = True
        if node.render_backend == 'matplotlib':
            try:
                if node.plt.fignum_exists(node.fig.number):
                    node.plt.close(node.fig)
            except Exception:
                pass
        else:
            try:
                pygame.quit()
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
