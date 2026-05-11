from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QMouseEvent, QPixmap
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


class CameraView(QLabel):
    clicked = Signal(int, int)

    def __init__(self) -> None:
        super().__init__("Camera stopped")
        self.setObjectName("cameraView")
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(640, 360)
        self.setScaledContents(False)
        self._frame_size: tuple[int, int] | None = None

    def set_frame(self, pixmap: QPixmap, frame_size: tuple[int, int]) -> None:
        self._frame_size = frame_size
        self.setPixmap(pixmap.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def mousePressEvent(self, event: QMouseEvent) -> None:
        if self.pixmap() is None or self._frame_size is None:
            return
        pixmap = self.pixmap()
        x_offset = max(0, (self.width() - pixmap.width()) // 2)
        y_offset = max(0, (self.height() - pixmap.height()) // 2)
        x = event.position().x() - x_offset
        y = event.position().y() - y_offset
        if x < 0 or y < 0 or x > pixmap.width() or y > pixmap.height():
            return
        frame_w, frame_h = self._frame_size
        self.clicked.emit(int(x / max(1, pixmap.width()) * frame_w), int(y / max(1, pixmap.height()) * frame_h))


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
