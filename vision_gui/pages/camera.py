from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSlider,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

import numpy as np

from ..detect import HsvRange


class CameraPage(QWidget):
    hsv_changed = Signal(object)       # HsvRange
    camera_index_changed = Signal(int)

    def __init__(self) -> None:
        super().__init__()
        layout = QHBoxLayout(self)
        layout.setSpacing(12)

        # ── left: video ──────────────────────────────────────────────────────
        left = QVBoxLayout()
        self._video = QLabel()
        self._video.setFixedSize(640, 480)
        self._video.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._video.setStyleSheet("background: #111; border: 1px solid #333;")
        self._video.setText("No camera")
        left.addWidget(self._video)

        self._det_label = QLabel("No detection")
        self._det_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        left.addWidget(self._det_label)
        layout.addLayout(left)

        # ── right: controls ──────────────────────────────────────────────────
        right = QVBoxLayout()
        right.setSpacing(12)

        hsv_group = QGroupBox("HSV Detection Tuning")
        hsv_layout = QVBoxLayout(hsv_group)

        self._lo_hue = self._slider("Lower hue", 0, 179, 40, hsv_layout)
        self._hi_hue = self._slider("Upper hue", 0, 179, 85, hsv_layout)
        self._sat    = self._slider("Sat min",   0, 255, 80, hsv_layout)
        self._val    = self._slider("Val min",   0, 255, 80, hsv_layout)
        right.addWidget(hsv_group)
        right.addStretch()
        layout.addLayout(right)

    # ── public ───────────────────────────────────────────────────────────────

    def update_frame(self, frame: np.ndarray) -> None:
        h, w, ch = frame.shape
        img = QImage(frame.data, w, h, ch * w, QImage.Format.Format_BGR888)
        self._video.setPixmap(QPixmap.fromImage(img).scaled(
            640, 480,
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        ))

    def set_detection_label(self, text: str) -> None:
        self._det_label.setText(text)

    def get_hsv(self) -> HsvRange:
        return HsvRange(
            lower_hue=self._lo_hue.value(),
            upper_hue=self._hi_hue.value(),
            sat_min=self._sat.value(),
            val_min=self._val.value(),
        )

    # ── private ──────────────────────────────────────────────────────────────

    def _slider(self, label: str, lo: int, hi: int, default: int, parent_layout: QVBoxLayout) -> QSlider:
        row = QHBoxLayout()
        lbl = QLabel(label)
        lbl.setFixedWidth(90)
        slider = QSlider(Qt.Orientation.Horizontal)
        slider.setRange(lo, hi)
        slider.setValue(default)
        val_lbl = QLabel(str(default))
        val_lbl.setFixedWidth(30)
        slider.valueChanged.connect(lambda v: val_lbl.setText(str(v)))
        slider.valueChanged.connect(lambda _: self.hsv_changed.emit(self.get_hsv()))
        row.addWidget(lbl)
        row.addWidget(slider)
        row.addWidget(val_lbl)
        parent_layout.addLayout(row)
        return slider
