from __future__ import annotations

from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QColor, QMouseEvent, QPainter, QPen, QPixmap, QPolygonF
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
        self._calibration_markers: dict[str, dict] = {}
        self._origin_pixel: tuple[int, int] | None = None
        self._expected_guides_enabled = False
        self._rulers_enabled = True
        self._grid_enabled = False
        self._grid_homography: list[list[float]] | None = None
        self._grid_workspace: dict | None = None
        self._side_table_line: dict | None = None
        self._side_samples: list[dict] = []
        self._side_board_markers: list[dict] = []
        self._side_board: dict | None = None
        self._side_fit: dict | None = None

    def set_frame(self, pixmap: QPixmap, frame_size: tuple[int, int]) -> None:
        self._frame_size = frame_size
        self._source_pixmap = pixmap
        self._render_scaled()

    def set_marker(self, pixel_x: int | None, pixel_y: int | None, text: str = "") -> None:
        self._marker = None if pixel_x is None or pixel_y is None else {"x": int(pixel_x), "y": int(pixel_y), "text": text}
        self._render_scaled()

    def set_calibration_overlay(
        self,
        *,
        markers: dict[str, dict] | None = None,
        origin_pixel: tuple[int, int] | None = None,
        expected_guides: bool = False,
        rulers_enabled: bool = True,
    ) -> None:
        self._calibration_markers = markers or {}
        self._origin_pixel = origin_pixel
        self._expected_guides_enabled = expected_guides
        self._rulers_enabled = rulers_enabled
        self._render_scaled()

    def set_grid_overlay(self, enabled: bool, homography: list[list[float]] | None = None, workspace: dict | None = None) -> None:
        self._grid_enabled = enabled
        self._grid_homography = homography
        self._grid_workspace = workspace
        self._render_scaled()

    def set_side_overlay(
        self,
        *,
        table_line: dict | None = None,
        samples: list[dict] | None = None,
        board_markers: list[dict] | None = None,
        board: dict | None = None,
        fit: dict | None = None,
    ) -> None:
        self._side_table_line = table_line
        self._side_samples = list(samples or [])
        self._side_board_markers = list(board_markers or [])
        self._side_board = board
        self._side_fit = fit
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
        if self._side_table_line or self._side_samples or self._side_board_markers or self._side_board or self._side_fit:
            self._draw_side_overlay(painter, display_rect)
        if self._expected_guides_enabled or self._calibration_markers:
            self._draw_calibration_overlay(painter, display_rect)
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

    def _draw_calibration_overlay(self, painter: QPainter, display_rect: QRectF) -> None:
        if self._expected_guides_enabled:
            self._draw_expected_guides(painter, display_rect)

        origin = None
        if self._origin_pixel:
            origin = self._frame_to_display(self._origin_pixel[0], self._origin_pixel[1], display_rect)
            painter.setPen(QPen(QColor("#38bdf8"), 3))
            painter.drawEllipse(origin, 9, 9)
            painter.drawText(QPointF(origin.x() + 12, origin.y() - 10), "Robot origin")

        for marker in self._calibration_markers.values():
            point = self._marker_center(marker, display_rect)
            if point is None:
                continue
            if self._rulers_enabled and origin is not None:
                pen = QPen(QColor(255, 255, 255, 180), 2)
                pen.setStyle(Qt.DashLine)
                painter.setPen(pen)
                painter.drawLine(origin, point)
                robot = marker.get("robot", {})
                mid = QPointF((origin.x() + point.x()) / 2.0, (origin.y() + point.y()) / 2.0)
                painter.setPen(QPen(QColor("#e2e8f0"), 1))
                painter.drawText(QPointF(mid.x() + 8, mid.y() - 8), f"X {float(robot.get('x', 0.0)):.0f}, Y {float(robot.get('y', 0.0)):.0f}")
            self._draw_calibration_marker(painter, display_rect, marker, point)

    def _draw_expected_guides(self, painter: QPainter, display_rect: QRectF) -> None:
        guides = {
            "BL": QPointF(display_rect.width() * 0.08, display_rect.height() * 0.10),
            "BR": QPointF(display_rect.width() * 0.88, display_rect.height() * 0.10),
            "FL": QPointF(display_rect.width() * 0.08, display_rect.height() * 0.88),
            "FR": QPointF(display_rect.width() * 0.88, display_rect.height() * 0.88),
        }
        painter.setPen(QPen(QColor(125, 211, 252, 150), 2))
        for short, point in guides.items():
            painter.drawEllipse(point, 13, 13)
            painter.drawText(QPointF(point.x() + 18, point.y() + 4), f"{short} expected")

    def _marker_center(self, marker: dict, display_rect: QRectF) -> QPointF | None:
        pixel = marker.get("pixel")
        if not isinstance(pixel, dict):
            return None
        try:
            return self._frame_to_display(float(pixel["x"]), float(pixel["y"]), display_rect)
        except Exception:
            return None

    def _draw_calibration_marker(self, painter: QPainter, display_rect: QRectF, marker: dict, point: QPointF) -> None:
        corners = marker.get("corners")
        if isinstance(corners, list) and len(corners) >= 4:
            polygon = QPolygonF()
            display_corners: list[QPointF] = []
            for corner in corners:
                if isinstance(corner, dict):
                    corner_point = self._frame_to_display(float(corner.get("x", 0.0)), float(corner.get("y", 0.0)), display_rect)
                    display_corners.append(corner_point)
                    polygon.append(corner_point)
            if polygon.count() >= 4:
                painter.setPen(QPen(QColor("#fbbf24"), 3))
                painter.drawPolygon(polygon)
                painter.setPen(QPen(QColor("#ffffff"), 2))
                for corner_point in display_corners:
                    painter.drawEllipse(corner_point, 3, 3)
        color = QColor("#22c55e") if marker.get("source") == "aruco" else QColor("#fb7185") if marker.get("source") == "manual" else QColor("#38bdf8")
        painter.setPen(QPen(color, 3))
        painter.drawLine(QPointF(point.x() - 14, point.y()), QPointF(point.x() + 14, point.y()))
        painter.drawLine(QPointF(point.x(), point.y() - 14), QPointF(point.x(), point.y() + 14))
        painter.drawEllipse(point, 8, 8)
        short = str(marker.get("short") or marker.get("label") or "?")
        source = str(marker.get("source") or "marker")
        robot = marker.get("robot", {})
        painter.setPen(QPen(QColor("#ffffff"), 1))
        painter.drawText(
            QPointF(min(point.x() + 16, display_rect.width() - 180), max(22, point.y() - 16)),
            f"{short} {source}  X {float(robot.get('x', 0.0)):.0f} Y {float(robot.get('y', 0.0)):.0f}",
        )

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

    def _draw_side_overlay(self, painter: QPainter, display_rect: QRectF) -> None:
        if self._side_board:
            outline = self._side_board.get("boardOutline")
            if isinstance(outline, list) and len(outline) >= 4:
                polygon = QPolygonF()
                for point in outline:
                    if isinstance(point, dict):
                        polygon.append(self._frame_to_display(float(point.get("x", 0.0)), float(point.get("y", 0.0)), display_rect))
                if polygon.count() >= 4:
                    painter.setPen(QPen(QColor("#22d3ee"), 4))
                    painter.drawPolygon(polygon)
            axes = self._side_board.get("axes")
            if isinstance(axes, dict):
                try:
                    origin = self._frame_to_display(float(axes["origin"]["x"]), float(axes["origin"]["y"]), display_rect)
                    x_axis = self._frame_to_display(float(axes["x"]["x"]), float(axes["x"]["y"]), display_rect)
                    y_axis = self._frame_to_display(float(axes["y"]["x"]), float(axes["y"]["y"]), display_rect)
                    painter.setPen(QPen(QColor("#ef4444"), 4))
                    painter.drawLine(origin, x_axis)
                    painter.drawText(x_axis, "X")
                    painter.setPen(QPen(QColor("#22c55e"), 4))
                    painter.drawLine(origin, y_axis)
                    painter.drawText(y_axis, "Y")
                except Exception:
                    pass
            for corner in self._side_board.get("charucoCorners", []) if isinstance(self._side_board.get("charucoCorners"), list) else []:
                if isinstance(corner, dict):
                    point = self._frame_to_display(float(corner.get("x", 0.0)), float(corner.get("y", 0.0)), display_rect)
                    painter.setPen(QPen(QColor("#ffffff"), 2))
                    painter.drawEllipse(point, 3, 3)

        for marker in self._side_board_markers:
            corners = marker.get("corners") if isinstance(marker, dict) else None
            if not isinstance(corners, list) or len(corners) < 4:
                continue
            polygon = QPolygonF()
            for corner in corners:
                if isinstance(corner, dict):
                    polygon.append(self._frame_to_display(float(corner.get("x", 0.0)), float(corner.get("y", 0.0)), display_rect))
            if polygon.count() >= 4:
                painter.setPen(QPen(QColor("#fde047"), 3))
                painter.drawPolygon(polygon)

        if self._side_table_line:
            p1 = self._side_table_line.get("p1", {})
            p2 = self._side_table_line.get("p2", {})
            try:
                start = self._frame_to_display(float(p1["x"]), float(p1["y"]), display_rect)
                end = self._frame_to_display(float(p2["x"]), float(p2["y"]), display_rect)
                painter.setPen(QPen(QColor("#38bdf8"), 3))
                painter.drawLine(start, end)
                painter.drawText(QPointF(start.x() + 10, start.y() - 10), "table line")
            except Exception:
                pass

        for index, sample in enumerate(self._side_samples, start=1):
            pixel = sample.get("pixel") if isinstance(sample, dict) else None
            if not isinstance(pixel, dict):
                continue
            try:
                point = self._frame_to_display(float(pixel["x"]), float(pixel["y"]), display_rect)
            except Exception:
                continue
            painter.setPen(QPen(QColor("#22c55e"), 3))
            painter.drawEllipse(point, 7, 7)
            painter.drawLine(QPointF(point.x() - 10, point.y()), QPointF(point.x() + 10, point.y()))
            painter.drawLine(QPointF(point.x(), point.y() - 10), QPointF(point.x(), point.y() + 10))
            painter.setPen(QPen(QColor("#ffffff"), 1))
            painter.drawText(QPointF(min(point.x() + 12, display_rect.width() - 160), max(18, point.y() - 12)), f"S{index} Z {float(sample.get('robotZ', 0.0)):.1f}")

        if self._side_fit:
            fit = self._side_fit.get("fit", {}) if isinstance(self._side_fit, dict) else {}
            z = self._side_fit.get("z") if isinstance(self._side_fit, dict) else None
            if isinstance(z, (int, float)):
                painter.setPen(QPen(QColor("#f97316"), 2))
                painter.drawText(QPointF(12, 24), f"visual tableZ {float(z):.1f} mm  fit error {float(fit.get('errorPx', 0.0)):.1f}px")


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
