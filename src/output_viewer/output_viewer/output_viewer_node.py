#!/usr/bin/env python3

from custom_interfaces.msg import HapticGrid
from custom_interfaces.msg import PipelineMode
from custom_interfaces.msg import SpatialAwareness
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import Float32

import math
import time

import numpy as np

import rclpy
from rclpy.node import Node


BG = (5, 8, 16)
FG = (166, 220, 238)
GRID = (34, 73, 112)
INVALID_BG = (13, 20, 38)
INVALID_FG = (74, 105, 126)
PANEL_BG = (8, 13, 25)
PANEL_EDGE = (72, 157, 210)
PANEL_EDGE_DIM = (30, 69, 105)
ACCENT = (108, 201, 242)
ACCENT_SOFT = (45, 117, 176)
HOT = (117, 238, 255)
TEXT_DARK = (7, 20, 34)
pygame = None


def extract_intensity_and_active(msg: HapticGrid):
    rows = int(msg.rows)
    cols = int(msg.cols)

    m = np.array([float(v) for v in msg.intensities], dtype=np.float32)
    active = np.array([bool(v) for v in msg.active], dtype=bool)

    if m.size != rows * cols or active.size != rows * cols:
        raise RuntimeError(
            f'Tamaño inconsistente: rows*cols={rows*cols} '
            f'pero llegó intensities={m.size} active={active.size}'
        )

    m = np.clip(m, 0.0, 100.0)
    cnt = active.astype(np.int32)
    return m.reshape((rows, cols)), cnt.reshape((rows, cols))


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


def intensity_to_neon_rgb(m: np.ndarray) -> np.ndarray:
    """Paleta monocromatica azul/cian para la vista pygame."""
    x = np.clip(m.astype(np.float32) / 100.0, 0.0, 1.0)
    x = np.power(x, 0.72)

    low = np.array([8, 13, 25], dtype=np.float32)
    mid = np.array([12, 64, 119], dtype=np.float32)
    high = np.array([22, 168, 224], dtype=np.float32)
    peak = np.array([102, 238, 255], dtype=np.float32)

    stops = np.stack((low, mid, high, peak), axis=0)
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

        self.declare_parameter('topic', '/perception/haptic_grid')
        self.declare_parameter('show_values', True)
        self.declare_parameter('invalid_text', '--')
        self.declare_parameter('refresh_hz', 20.0)
        self.declare_parameter('render_backend', 'pygame')
        self.declare_parameter('flip_rows_for_display', False)
        self.declare_parameter('pygame_view_mode', 'curved')
        self.declare_parameter('pygame_maximized', True)
        self.declare_parameter('aperture_control_enabled', True)
        self.declare_parameter(
            'aperture_command_topic',
            '/perception/depth_grid/aperture_deg',
        )
        self.declare_parameter('use_parameter_service_control', False)
        self.declare_parameter('encoder_node_name', '/obstacle_grid_encoder')
        self.declare_parameter('h_aperture_deg', 45.0)
        self.declare_parameter('h_aperture_min_deg', 15.0)
        self.declare_parameter('h_aperture_max_deg', 70.0)
        self.declare_parameter(
            'spatial_awareness_topic',
            '/spatial_awareness/collision_risk',
        )
        self.declare_parameter('show_spatial_awareness', True)
        self.declare_parameter('spatial_awareness_timeout_s', 0.75)
        self.declare_parameter('pipeline_selected_topic', '/pipeline/selected_mode')
        self.declare_parameter('pipeline_command_topic', '/pipeline/selected_mode_cmd')

        self.topic = str(self.get_parameter('topic').value)
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
        self.pygame_maximized = bool(
            self.get_parameter('pygame_maximized').value
        )
        self.aperture_control_enabled = bool(
            self.get_parameter('aperture_control_enabled').value
        )
        self.aperture_command_topic = str(
            self.get_parameter('aperture_command_topic').value
        )
        self.use_parameter_service_control = bool(
            self.get_parameter('use_parameter_service_control').value
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
        self.spatial_awareness_topic = str(
            self.get_parameter('spatial_awareness_topic').value
        )
        self.show_spatial_awareness = bool(
            self.get_parameter('show_spatial_awareness').value
        )
        self.spatial_awareness_timeout_s = float(
            self.get_parameter('spatial_awareness_timeout_s').value
        )
        self.pipeline_selected_topic = str(
            self.get_parameter('pipeline_selected_topic').value
        )
        self.pipeline_command_topic = str(
            self.get_parameter('pipeline_command_topic').value
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

        self.sub = self.create_subscription(
            HapticGrid,
            self.topic,
            self.cb_haptic_grid,
            10,
        )
        self.spatial_awareness_sub = self.create_subscription(
            SpatialAwareness,
            self.spatial_awareness_topic,
            self.cb_spatial_awareness,
            10,
        )
        self.pipeline_selected_sub = self.create_subscription(
            PipelineMode,
            self.pipeline_selected_topic,
            self.cb_pipeline_mode,
            10,
        )
        self.pipeline_mode_pub = self.create_publisher(
            PipelineMode,
            self.pipeline_command_topic,
            10,
        )

        # Estado compartido (actualizado por el callback)
        self.latest_m = None     # intensidad 0..100
        self.latest_cnt = None   # count
        self.latest_shape = None
        self.spatial_awareness = {
            'left': 0.0,
            'right': 0.0,
            'rear': 0.0,
        }
        self.spatial_awareness_last_rx_s = 0.0
        self.spatial_awareness_valid = False
        self.pipeline_mode = int(PipelineMode.MODE_RAW)
        self._pipeline_buttons = []
        self._dirty = False
        self._dragging_aperture = False
        self._last_aperture_send_s = 0.0
        self._aperture_pending = False
        self._aperture_pub = None
        self._aperture_client = None
        if self.aperture_control_enabled:
            self._aperture_pub = self.create_publisher(
                Float32,
                self.aperture_command_topic,
                10,
            )
            self._aperture_pending = True
            if self.use_parameter_service_control:
                service_name = f'{self.encoder_node_name}/set_parameters'
                self._aperture_client = self.create_client(
                    SetParameters,
                    service_name,
                )

        self._closing = False
        if self.render_backend == 'matplotlib':
            self._setup_matplotlib()
        else:
            self._setup_pygame()

        # Timer ROS para refrescar el plot
        period = 1.0 / max(self.refresh_hz, 1.0)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            f'Escuchando {self.topic} | intensity grid '
            f'| refresh={self.refresh_hz}Hz '
            f'| backend={self.render_backend} '
            f'| flip_rows={self.flip_rows_for_display} '
            f'| pygame_view={self.pygame_view_mode} '
            f'| aperture={self.h_aperture_deg:.1f}deg '
            f'| spatial_awareness={self.spatial_awareness_topic} '
            f'| pipeline_mode={self.pipeline_selected_topic}'
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
        if self.pygame_maximized:
            info = pygame.display.Info()
            self.screen = pygame.display.set_mode(
                (info.current_w, info.current_h),
                pygame.RESIZABLE,
            )
        else:
            self.screen = pygame.display.set_mode((900, 700), pygame.RESIZABLE)
        pygame.display.set_caption(
            f'Depth Grid Tactical View: {self.topic}'
        )
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont('dejavusans', 18, bold=True)
        self.title_font = pygame.font.SysFont('dejavusans', 34, bold=True)
        self.value_font = pygame.font.SysFont('dejavusans', 18, bold=True)
        self.small_font = pygame.font.SysFont('dejavusans', 14, bold=True)
        self._draw_waiting()

    def _on_close(self, _evt=None):
        # Usuario cerró la ventana: marcamos cierre y apagamos ROS
        self._closing = True
        try:
            rclpy.shutdown()
        except Exception:
            pass

    def cb_haptic_grid(self, msg: HapticGrid):
        try:
            m, cnt = extract_intensity_and_active(msg)
        except Exception as e:
            self.get_logger().error(f'No pude parsear HapticGrid: {e}')
            return

        self.latest_m = m
        self.latest_cnt = cnt
        self.latest_shape = m.shape
        self._dirty = True

    def cb_spatial_awareness(self, msg: SpatialAwareness):
        self.spatial_awareness_valid = bool(msg.valid)
        self.spatial_awareness['left'] = self._risk_intensity(msg.left)
        self.spatial_awareness['right'] = self._risk_intensity(msg.right)
        self.spatial_awareness['rear'] = self._risk_intensity(msg.rear)
        self.spatial_awareness_last_rx_s = time.monotonic()
        self._dirty = True

    def cb_pipeline_mode(self, msg: PipelineMode):
        mode = int(msg.mode)
        if mode in (
            int(PipelineMode.MODE_RAW),
            int(PipelineMode.MODE_FILTERED),
            int(PipelineMode.MODE_FULL),
        ):
            if mode != self.pipeline_mode:
                self.pipeline_mode = mode
                self._dirty = True

    def _publish_pipeline_mode(self, mode: int):
        if mode not in (
            int(PipelineMode.MODE_RAW),
            int(PipelineMode.MODE_FILTERED),
            int(PipelineMode.MODE_FULL),
        ):
            return
        msg = PipelineMode()
        msg.mode = int(mode)
        self.pipeline_mode_pub.publish(msg)

    def _pipeline_mode_label(self):
        if self.pipeline_mode == int(PipelineMode.MODE_FILTERED):
            return 'FILTERED'
        if self.pipeline_mode == int(PipelineMode.MODE_FULL):
            return 'FULL'
        return 'RAW'

    def _pipeline_buttons_layout(self):
        width, _height = self.screen.get_size()
        group_w = min(540, max(300, width - 260))
        btn_gap = 14
        btn_w = (group_w - 2 * btn_gap) // 3
        btn_h = 34
        x0 = (width - group_w) // 2
        y = 214
        return [
            ('RAW', int(PipelineMode.MODE_RAW), pygame.Rect(x0, y, btn_w, btn_h)),
            (
                'FILTERED',
                int(PipelineMode.MODE_FILTERED),
                pygame.Rect(x0 + btn_w + btn_gap, y, btn_w, btn_h),
            ),
            (
                'FULL',
                int(PipelineMode.MODE_FULL),
                pygame.Rect(x0 + 2 * (btn_w + btn_gap), y, btn_w, btn_h),
            ),
        ]

    def _draw_pipeline_buttons(self):
        self._pipeline_buttons = self._pipeline_buttons_layout()
        self._draw_text(
            f'PIPELINE MODE: {self._pipeline_mode_label()}',
            (self.screen.get_width() // 2, 196),
            self.small_font,
            FG,
            center=True,
        )

        for label, mode, rect in self._pipeline_buttons:
            selected = mode == self.pipeline_mode
            fill = (20, 110, 82) if selected else (26, 40, 58)
            edge = (121, 231, 190) if selected else PANEL_EDGE_DIM
            text_color = HOT if selected else FG
            pygame.draw.rect(self.screen, fill, rect, border_radius=7)
            pygame.draw.rect(self.screen, edge, rect, 2, border_radius=7)
            self._draw_text(
                label,
                rect.center,
                self.small_font,
                text_color,
                center=True,
            )

    def _handle_pipeline_button_event(self, event):
        if event.type != pygame.MOUSEBUTTONDOWN or event.button != 1:
            return False

        for _label, mode, rect in self._pipeline_buttons:
            if rect.collidepoint(event.pos):
                self._publish_pipeline_mode(mode)
                return True
        return False

    def _risk_intensity(self, risk) -> float:
        if not self.spatial_awareness_valid or not risk.active:
            return 0.0
        return float(np.clip(risk.intensity, 0.0, 100.0))

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
        slider_w = min(440, max(260, width - 360))
        return pygame.Rect((width - slider_w) // 2, 164, slider_w, 8)

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
        now = time.monotonic()
        if not force and now - self._last_aperture_send_s < 0.15:
            return
        self._last_aperture_send_s = now

        if self._aperture_pub is not None:
            msg = Float32()
            msg.data = float(self.h_aperture_deg)
            self._aperture_pub.publish(msg)

        if self._aperture_client is None:
            self._aperture_pending = False
            return

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
            if self._handle_pipeline_button_event(event):
                continue
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
        self._draw_background()
        width, height = self.screen.get_size()
        self._draw_panel(
            pygame.Rect(34, 28, width - 68, height - 56),
            border=2,
        )
        self._draw_text(
            'DEPTH GRID',
            (width // 2, 58),
            self.title_font,
            FG,
            center=True,
            glow=True,
        )
        self._draw_text(
            f'{self.topic}  |  SIGNAL INTENSITY',
            (width // 2, 94),
            self.small_font,
            INVALID_FG,
            center=True,
        )
        self._draw_aperture_slider()
        self._draw_pipeline_buttons()
        self._draw_text(
            'WAITING FOR SIGNAL',
            (width // 2, height // 2),
            self.font,
            FG,
            center=True,
            glow=True,
        )
        self._draw_spatial_awareness_indicators()
        pygame.display.flip()

    def _draw_text(
        self,
        text: str,
        pos,
        font=None,
        color=FG,
        center=False,
        glow=False,
        shadow=False,
    ):
        font = font or self.font
        surf = font.render(text, True, color)
        rect = surf.get_rect()
        if center:
            rect.center = pos
        else:
            rect.topleft = pos
        if shadow:
            shadow_surf = font.render(text, True, (0, 4, 12))
            self.screen.blit(shadow_surf, rect.move(1, 1))
        if glow:
            glow_surf = font.render(text, True, ACCENT_SOFT)
            for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                self.screen.blit(glow_surf, rect.move(dx, dy))
        self.screen.blit(surf, rect)
        return rect

    def _draw_background(self):
        width, height = self.screen.get_size()
        for y in range(height):
            t = y / max(1, height - 1)
            color = (
                int(3 + 5 * t),
                int(7 + 9 * t),
                int(18 + 22 * t),
            )
            pygame.draw.line(self.screen, color, (0, y), (width, y))

        grid_step = 48
        for x in range(0, width, grid_step):
            pygame.draw.line(self.screen, (8, 25, 55), (x, 0), (x, height), 1)
        for y in range(0, height, grid_step):
            pygame.draw.line(self.screen, (8, 25, 55), (0, y), (width, y), 1)

    def _draw_panel(self, rect, border=1):
        shadow = pygame.Surface((rect.width + 28, rect.height + 28), pygame.SRCALPHA)
        pygame.draw.rect(
            shadow,
            (*ACCENT_SOFT, 34),
            shadow.get_rect().inflate(-18, -18),
            border_radius=2,
        )
        self.screen.blit(shadow, (rect.left - 14, rect.top - 14))

        panel = pygame.Surface(rect.size, pygame.SRCALPHA)
        pygame.draw.rect(panel, (*PANEL_BG, 232), panel.get_rect(), border_radius=8)
        self.screen.blit(panel, rect)
        pygame.draw.rect(self.screen, PANEL_EDGE_DIM, rect, border, border_radius=8)
        pygame.draw.rect(self.screen, PANEL_EDGE, rect.inflate(-7, -7), 1, border_radius=6)

        length = min(62, max(22, rect.width // 8))
        for sx in (rect.left, rect.right):
            x0 = sx if sx == rect.left else sx - length
            x1 = sx + length if sx == rect.left else sx
            pygame.draw.line(self.screen, ACCENT, (x0, rect.top), (x1, rect.top), 2)
            pygame.draw.line(self.screen, ACCENT, (x0, rect.bottom), (x1, rect.bottom), 2)
        for sy in (rect.top, rect.bottom):
            y0 = sy if sy == rect.top else sy - length
            y1 = sy + length if sy == rect.top else sy
            pygame.draw.line(self.screen, ACCENT, (rect.left, y0), (rect.left, y1), 2)
            pygame.draw.line(self.screen, ACCENT, (rect.right, y0), (rect.right, y1), 2)

    def _draw_aperture_slider(self):
        if not self.aperture_control_enabled:
            return

        rect = self._slider_rect()
        label = f'APERTURE ±{self.h_aperture_deg:.0f}°'
        self._draw_text(
            label,
            (rect.centerx, rect.top - 30),
            self.font,
            FG,
            center=True,
        )

        pygame.draw.line(
            self.screen,
            PANEL_EDGE_DIM,
            rect.midleft,
            rect.midright,
            3,
        )
        t = (
            (self.h_aperture_deg - self.h_aperture_min_deg) /
            max(1e-3, self.h_aperture_max_deg - self.h_aperture_min_deg)
        )
        knob_x = rect.left + int(round(t * rect.width))
        pygame.draw.line(
            self.screen,
            ACCENT,
            rect.midleft,
            (knob_x, rect.centery),
            3,
        )
        pygame.draw.circle(self.screen, ACCENT_SOFT, (knob_x, rect.centery), 10)
        pygame.draw.circle(self.screen, HOT, (knob_x, rect.centery), 6)

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
        margin = 34
        title_h = 152
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
        max_abs_h = math.radians(max(self.h_aperture_deg, 1.0))
        x_norm = math.sin(h_angle) / max(math.sin(max_abs_h), 1e-3)
        x = rect.centerx + x_norm * rect.width * 0.50

        curve_norm = (1.0 - math.cos(h_angle)) / max(
            1.0 - math.cos(max_abs_h),
            1e-3,
        )
        y = rect.top + row_pos * rect.height + curve_norm * 38.0
        return int(round(x)), int(round(y))

    def _cell_points_curved(self, plot_rect, h_edges, row_edges, view_r, col):
        return [
            self._project_curved(plot_rect, h_edges[col], row_edges[view_r]),
            self._project_curved(plot_rect, h_edges[col + 1], row_edges[view_r]),
            self._project_curved(plot_rect, h_edges[col + 1], row_edges[view_r + 1]),
            self._project_curved(plot_rect, h_edges[col], row_edges[view_r + 1]),
        ]

    def _draw_signal_dot(self, center, radius, color, intensity):
        radius = max(3, int(radius))
        glow_radius = int(radius * (1.65 + 0.35 * intensity / 100.0))
        glow_size = glow_radius * 2 + 8
        glow = pygame.Surface((glow_size, glow_size), pygame.SRCALPHA)
        glow_center = (glow_size // 2, glow_size // 2)

        pygame.draw.circle(
            glow,
            (*color, int(12 + intensity * 0.34)),
            glow_center,
            glow_radius,
        )
        if intensity >= 55.0:
            pygame.draw.circle(
                glow,
                (*HOT, int((intensity - 45.0) * 0.95)),
                glow_center,
                max(radius + 3, int(glow_radius * 0.52)),
            )
        self.screen.blit(
            glow,
            (center[0] - glow_center[0], center[1] - glow_center[1]),
            special_flags=pygame.BLEND_ADD,
        )

        pygame.draw.circle(self.screen, (5, 13, 24), center, radius + 2)
        pygame.draw.circle(self.screen, color, center, radius)

        inner = (
            min(150, color[0] + 24),
            min(245, color[1] + 26),
            min(255, color[2] + 28),
        )
        pygame.draw.circle(self.screen, inner, center, max(2, int(radius * 0.44)))
        pygame.draw.circle(self.screen, PANEL_EDGE, center, radius, 1)
        highlight = (
            center[0] - max(1, int(radius * 0.22)),
            center[1] - max(1, int(radius * 0.22)),
        )
        pygame.draw.circle(self.screen, HOT, highlight, max(1, radius // 7))

    def _spatial_awareness_is_fresh(self):
        if not self.spatial_awareness_valid:
            return False
        age_s = time.monotonic() - self.spatial_awareness_last_rx_s
        return age_s <= max(0.05, self.spatial_awareness_timeout_s)

    def _draw_spatial_awareness_indicators(self, plot_rect=None):
        if not self.show_spatial_awareness:
            return

        width, height = self.screen.get_size()
        fresh = self._spatial_awareness_is_fresh()
        radius = 20
        if plot_rect is None:
            y = height - 118
            positions = {
                'LEFT': (width // 2 - 96, y),
                'REAR': (width // 2, y),
                'RIGHT': (width // 2 + 96, y),
            }
        else:
            side_y = plot_rect.centery
            bottom_y = min(height - 78, plot_rect.bottom + 58)
            positions = {
                'LEFT': (max(70, plot_rect.left - 62), side_y),
                'REAR': (plot_rect.centerx, bottom_y),
                'RIGHT': (min(width - 70, plot_rect.right + 62), side_y),
            }

        self._draw_text(
            'SPATIAL',
            (positions['REAR'][0], positions['REAR'][1] - radius - 34),
            self.small_font,
            FG if fresh else INVALID_FG,
            center=True,
        )

        for label in ('LEFT', 'REAR', 'RIGHT'):
            intensity = self.spatial_awareness[label.lower()] if fresh else 0.0
            color = tuple(
                intensity_to_neon_rgb(
                    np.array([[intensity]], dtype=np.float32)
                )[0, 0]
            )
            if not fresh:
                color = PANEL_EDGE_DIM

            center = positions[label]
            self._draw_signal_dot(center, radius, color, intensity)
            self._draw_text(
                f'{int(round(intensity))}',
                center,
                self.value_font,
                TEXT_DARK if intensity >= 56 else HOT,
                center=True,
                shadow=intensity < 56,
            )
            self._draw_text(
                label,
                (center[0], center[1] + radius + 18),
                self.small_font,
                FG if fresh else INVALID_FG,
                center=True,
            )

    def _draw_heatmap_curved(self, m: np.ndarray, cnt: np.ndarray):
        rows, cols = m.shape
        width, height = self.screen.get_size()
        self._draw_background()
        main_panel = pygame.Rect(34, 28, width - 68, height - 56)
        self._draw_panel(main_panel, border=2)

        self._draw_text(
            'DEPTH GRID',
            (width // 2, 60),
            self.title_font,
            FG,
            center=True,
            glow=True,
        )
        self._draw_text(
            f'{self.topic}  |  SIGNAL INTENSITY',
            (width // 2, 96),
            self.small_font,
            INVALID_FG,
            center=True,
        )
        self._draw_aperture_slider()
        self._draw_pipeline_buttons()

        plot_rect, cbar_rect = self._layout(rows, cols)
        plot_rect = plot_rect.inflate(-24, -58)
        plot_rect.top += 64

        rgb = intensity_to_neon_rgb(m)
        h_edges = self._angle_edges(
            -self.h_aperture_deg,
            self.h_aperture_deg,
            cols,
        )
        row_edges = np.linspace(0.0, 1.0, rows + 1)
        cell_w = max(10, plot_rect.width / max(cols, 1))
        cell_h = max(10, plot_rect.height / max(rows, 1))
        max_radius = max(5, int(min(cell_w, cell_h) * 0.28))
        min_radius = max(3, int(max_radius * 0.58))

        for r in range(rows):
            view_r = display_row(rows, r, self.flip_rows_for_display)
            for c in range(cols):
                valid = cnt[r, c] > 0
                color = INVALID_BG if not valid else tuple(rgb[r, c])
                points = self._cell_points_curved(
                    plot_rect,
                    h_edges,
                    row_edges,
                    view_r,
                    c,
                )
                center = (
                    int(round(sum(p[0] for p in points) / 4.0)),
                    int(round(sum(p[1] for p in points) / 4.0)),
                )

                if valid:
                    v = float(np.clip(m[r, c], 0.0, 100.0))
                    radius = int(min_radius + (v / 100.0) * (max_radius - min_radius))
                    self._draw_signal_dot(center, radius, color, v)
                else:
                    radius = max(2, int(min_radius * 0.55))
                    pygame.draw.circle(self.screen, INVALID_FG, center, radius, 1)

                if self.show_values and valid:
                    txt = (
                        self.invalid_text
                        if not np.isfinite(v)
                        else f'{int(round(v))}'
                    )
                    value_color = TEXT_DARK if v >= 56 else HOT
                    self._draw_text(
                        txt,
                        center,
                        self.value_font,
                        value_color,
                        center=True,
                        shadow=v < 56,
                    )
                elif self.show_values:
                    self._draw_text(
                        self.invalid_text,
                        center,
                        self.value_font,
                        INVALID_FG,
                        center=True,
                    )

        near_arc = [
            self._project_curved(plot_rect, angle, row_edges[-1])
            for angle in np.linspace(h_edges[0], h_edges[-1], 48)
        ]
        pygame.draw.lines(self.screen, (42, 108, 166), False, near_arc, 1)

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
        gradient_rgb = intensity_to_neon_rgb(gradient)
        gradient_rgb = np.repeat(gradient_rgb, cbar_rect.width, axis=1)
        cbar_surf = pygame.surfarray.make_surface(
            np.transpose(gradient_rgb, (1, 0, 2))
        )
        self.screen.blit(cbar_surf, cbar_rect)
        pygame.draw.rect(self.screen, PANEL_EDGE, cbar_rect, 1)
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

        self._draw_spatial_awareness_indicators(plot_rect)
        pygame.display.flip()
        self.clock.tick(max(1.0, self.refresh_hz))

    def _draw_heatmap(self, m: np.ndarray, cnt: np.ndarray):
        if self.pygame_view_mode == 'curved':
            self._draw_heatmap_curved(m, cnt)
            return

        rows, cols = m.shape
        width, height = self.screen.get_size()
        self._draw_background()
        self._draw_panel(pygame.Rect(34, 28, width - 68, height - 56), border=2)

        self._draw_text(
            'DEPTH GRID',
            (width // 2, 60),
            self.title_font,
            FG,
            center=True,
            glow=True,
        )
        self._draw_aperture_slider()
        self._draw_pipeline_buttons()

        plot_rect, cbar_rect = self._layout(rows, cols)
        plot_rect = plot_rect.move(0, 42)
        rgb = intensity_to_neon_rgb(m)
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

        self._draw_spatial_awareness_indicators(plot_rect)
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
