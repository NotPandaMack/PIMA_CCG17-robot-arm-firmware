from __future__ import annotations

from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QColor, QMouseEvent, QPainter, QPen, QPixmap
from PySide6.QtWidgets import (
    QCheckBox,
    QFormLayout,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSlider,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from ..detection_worker import HSVProfile
from ..geometry import map_widget_point_to_frame


class CameraView(QLabel):
    clicked = Signal(int, int)
    mapped_clicked = Signal(object)

    def __init__(self) -> None:
        super().__init__("Camera stopped")
        self.setObjectName("cameraView")
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(640, 360)
        self.setScaledContents(False)
        self._frame_size: tuple[int, int] | None = None
        self._source_pixmap: QPixmap | None = None
        self._marker: dict | None = None
        self._grid_enabled = False
        self._grid_homography: list[list[float]] | None = None
        self._grid_workspace: dict | None = None

    def set_frame(self, pixmap: QPixmap, frame_size: tuple[int, int]) -> None:
        self._frame_size = frame_size
        self._source_pixmap = pixmap
        self._render_scaled()

    def set_marker(self, pixel_x: int | None, pixel_y: int | None, text: str = "") -> None:
        self._marker = None if pixel_x is None or pixel_y is None else {"x": int(pixel_x), "y": int(pixel_y), "text": text}
        self._render_scaled()

    def set_grid_overlay(self, enabled: bool, homography: list[list[float]] | None = None, workspace: dict | None = None) -> None:
        self._grid_enabled = enabled
        self._grid_homography = homography
        self._grid_workspace = workspace
        self._render_scaled()

    def resizeEvent(self, event: object) -> None:
        super().resizeEvent(event)
        self._render_scaled()

    def mousePressEvent(self, event: QMouseEvent) -> None:
        if self._frame_size is None:
            return
        frame_w, frame_h = self._frame_size
        mapped = map_widget_point_to_frame(
            widget_width=self.width(),
            widget_height=self.height(),
            frame_width=frame_w,
            frame_height=frame_h,
            widget_x=event.position().x(),
            widget_y=event.position().y(),
        )
        if mapped is None:
            return
        self.clicked.emit(mapped["pixelX"], mapped["pixelY"])
        self.mapped_clicked.emit(mapped)

    def _render_scaled(self) -> None:
        if self._source_pixmap is None or self._frame_size is None or self.width() <= 0 or self.height() <= 0:
            return
        scaled = self._source_pixmap.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        painter = QPainter(scaled)
        display_rect = QRectF(0, 0, scaled.width(), scaled.height())
        if self._grid_enabled and self._grid_homography and self._grid_workspace:
            self._draw_grid(painter, display_rect)
        if self._marker:
            self._draw_marker(painter, display_rect)
        painter.end()
        self.setPixmap(scaled)

    def _frame_to_display(self, pixel_x: float, pixel_y: float, display_rect: QRectF) -> QPointF:
        frame_w, frame_h = self._frame_size or (1, 1)
        return QPointF((pixel_x / frame_w) * display_rect.width(), (pixel_y / frame_h) * display_rect.height())

    def _draw_marker(self, painter: QPainter, display_rect: QRectF) -> None:
        point = self._frame_to_display(self._marker["x"], self._marker["y"], display_rect)
        pen = QPen(QColor("#ff4d5e"), 3)
        painter.setPen(pen)
        painter.drawLine(QPointF(point.x() - 12, point.y()), QPointF(point.x() + 12, point.y()))
        painter.drawLine(QPointF(point.x(), point.y() - 12), QPointF(point.x(), point.y() + 12))
        painter.drawEllipse(point, 7, 7)
        text = self._marker.get("text", "")
        if text:
            painter.setPen(QPen(QColor("#ffffff"), 1))
            painter.drawText(QPointF(min(point.x() + 14, display_rect.width() - 260), max(20, point.y() - 14)), text)

    def _draw_grid(self, painter: QPainter, display_rect: QRectF) -> None:
        points = self._grid_points()
        if not points:
            return
        painter.setPen(QPen(QColor(100, 180, 255, 150), 1))
        for pixel_x, pixel_y, label in points:
            point = self._frame_to_display(pixel_x, pixel_y, display_rect)
            painter.drawEllipse(point, 2, 2)
            painter.drawText(QPointF(point.x() + 4, point.y() - 4), label)

    def _grid_points(self) -> list[tuple[float, float, str]]:
        try:
            import numpy as np

            h = np.array(self._grid_homography, dtype=float)
            inv = np.linalg.inv(h)
            bounds = self._grid_workspace or {}
            xs = _grid_values(float(bounds.get("xMin", -180.0)), float(bounds.get("xMax", 180.0)), 60.0)
            ys = _grid_values(float(bounds.get("yMin", 60.0)), float(bounds.get("yMax", 285.0)), 60.0)
            frame_w, frame_h = self._frame_size or (0, 0)
            points = []
            for robot_x in xs:
                for robot_y in ys:
                    denom = (inv[2][0] * robot_x) + (inv[2][1] * robot_y) + inv[2][2]
                    if abs(denom) < 1e-9:
                        continue
                    pixel_x = ((inv[0][0] * robot_x) + (inv[0][1] * robot_y) + inv[0][2]) / denom
                    pixel_y = ((inv[1][0] * robot_x) + (inv[1][1] * robot_y) + inv[1][2]) / denom
                    if 0 <= pixel_x < frame_w and 0 <= pixel_y < frame_h:
                        points.append((float(pixel_x), float(pixel_y), f"{robot_x:.0f},{robot_y:.0f}"))
            return points
        except Exception:
            return []


def _grid_values(minimum: float, maximum: float, step: float) -> list[float]:
    values = []
    current = minimum
    while current <= maximum + 1e-6:
        values.append(current)
        current += step
    if values and abs(values[-1] - maximum) > 1e-6:
        values.append(maximum)
    return values


class CameraPage(QWidget):
    start_camera_requested = Signal()
    stop_camera_requested = Signal()
    send_target_requested = Signal()
    save_hsv_requested = Signal()
    reset_hsv_requested = Signal()
    profile_changed = Signal(object)
    continuous_changed = Signal(bool)
    rate_changed = Signal(float)

    def __init__(self) -> None:
        super().__init__()
        layout = QGridLayout(self)
        layout.setSpacing(14)
        self.camera_view = CameraView()
        layout.addWidget(self.camera_view, 0, 0, 3, 1)

        controls = QFrame()
        controls.setObjectName("panel")
        controls_layout = QVBoxLayout(controls)
        controls_layout.addWidget(QLabel("Camera / Object Detection"))

        button_row = QHBoxLayout()
        self.start_button = QPushButton("Start Camera")
        self.stop_button = QPushButton("Stop Camera")
        self.send_button = QPushButton("Send Detected Target to Pi")
        self.send_button.setObjectName("primaryButton")
        for button in (self.start_button, self.stop_button, self.send_button):
            button_row.addWidget(button)
        controls_layout.addLayout(button_row)

        self.continuous = QCheckBox("Continuously send target to Pi")
        self.rate = QSpinBox()
        self.rate.setRange(1, 15)
        self.rate.setValue(5)
        controls_layout.addWidget(self.continuous)
        rate_form = QFormLayout()
        rate_form.addRow("Send rate limit (Hz)", self.rate)
        controls_layout.addLayout(rate_form)

        self.detection_label = QLabel("No object detected.")
        self.robot_label = QLabel("Object detected in camera pixels only. Calibration required before robot movement.")
        self.robot_label.setWordWrap(True)
        controls_layout.addWidget(self.detection_label)
        controls_layout.addWidget(self.robot_label)

        hsv_box = QFrame()
        hsv_box.setObjectName("subPanel")
        hsv_form = QFormLayout(hsv_box)
        self.lower_hue = self._slider(0, 179, 40)
        self.upper_hue = self._slider(0, 179, 85)
        self.sat_min = self._slider(0, 255, 80)
        self.val_min = self._slider(0, 255, 80)
        self.min_area = QSpinBox()
        self.min_area.setRange(10, 50000)
        self.min_area.setValue(350)
        hsv_form.addRow("Lower hue", self.lower_hue)
        hsv_form.addRow("Upper hue", self.upper_hue)
        hsv_form.addRow("Saturation min", self.sat_min)
        hsv_form.addRow("Value min", self.val_min)
        hsv_form.addRow("Minimum area", self.min_area)
        controls_layout.addWidget(hsv_box)

        hsv_buttons = QHBoxLayout()
        self.reset_hsv_button = QPushButton("Reset HSV Defaults")
        self.save_hsv_button = QPushButton("Save HSV Profile")
        hsv_buttons.addWidget(self.reset_hsv_button)
        hsv_buttons.addWidget(self.save_hsv_button)
        controls_layout.addLayout(hsv_buttons)
        controls_layout.addStretch(1)

        layout.addWidget(controls, 0, 1)
        layout.setColumnStretch(0, 3)
        layout.setColumnStretch(1, 1)

        self.start_button.clicked.connect(self.start_camera_requested.emit)
        self.stop_button.clicked.connect(self.stop_camera_requested.emit)
        self.send_button.clicked.connect(self.send_target_requested.emit)
        self.save_hsv_button.clicked.connect(self.save_hsv_requested.emit)
        self.reset_hsv_button.clicked.connect(self.reset_hsv_requested.emit)
        self.continuous.toggled.connect(self.continuous_changed.emit)
        self.rate.valueChanged.connect(lambda value: self.rate_changed.emit(float(value)))
        for slider in (self.lower_hue, self.upper_hue, self.sat_min, self.val_min):
            slider.valueChanged.connect(self._emit_profile)
        self.min_area.valueChanged.connect(self._emit_profile)

    def set_profile(self, profile: HSVProfile) -> None:
        self.lower_hue.setValue(profile.lower_hue)
        self.upper_hue.setValue(profile.upper_hue)
        self.sat_min.setValue(profile.saturation_min)
        self.val_min.setValue(profile.value_min)
        self.min_area.setValue(int(profile.min_area))

    def profile(self) -> HSVProfile:
        return HSVProfile(
            lower_hue=self.lower_hue.value(),
            upper_hue=self.upper_hue.value(),
            saturation_min=self.sat_min.value(),
            value_min=self.val_min.value(),
            min_area=float(self.min_area.value()),
        )

    def update_detection(self, text: str, robot_text: str, can_send: bool) -> None:
        self.detection_label.setText(text)
        self.robot_label.setText(robot_text)
        self.send_button.setEnabled(can_send)

    def _emit_profile(self) -> None:
        self.profile_changed.emit(self.profile())

    @staticmethod
    def _slider(minimum: int, maximum: int, value: int) -> QSlider:
        slider = QSlider(Qt.Horizontal)
        slider.setRange(minimum, maximum)
        slider.setValue(value)
        slider.setTickPosition(QSlider.TicksBelow)
        slider.setTickInterval(max(1, (maximum - minimum) // 6))
        return slider
