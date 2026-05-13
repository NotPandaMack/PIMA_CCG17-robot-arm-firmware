from __future__ import annotations

import time
from typing import Any

import numpy as np
from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (
    QCheckBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from ..detect import Detection, HsvRange


_AUTO_STABLE_SEC = 1.5    # object must be detected for this long before auto-pick fires
_AUTO_COOLDOWN_SEC = 4.0  # minimum gap between auto-picks


class PickPage(QWidget):
    hover_requested = Signal()
    pick_requested = Signal()
    enable_motion_requested = Signal(bool)
    status_message = Signal(str)

    def __init__(self) -> None:
        super().__init__()
        layout = QHBoxLayout(self)
        layout.setSpacing(12)

        # ── left: live view ──────────────────────────────────────────────────
        left = QVBoxLayout()
        self._video = QLabel()
        self._video.setFixedSize(640, 480)
        self._video.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._video.setStyleSheet("background: #111; border: 1px solid #333;")
        self._video.setText("No camera")
        left.addWidget(self._video)
        self._det_label = QLabel("No object detected")
        self._det_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        left.addWidget(self._det_label)
        layout.addLayout(left)

        # ── right: controls ──────────────────────────────────────────────────
        right = QVBoxLayout()
        right.setSpacing(12)

        # target readout
        target_group = QGroupBox("Current Detection")
        target_form = QFormLayout(target_group)
        self._robot_xy_label = QLabel("--")
        self._conf_label = QLabel("--")
        target_form.addRow("Robot X, Y:", self._robot_xy_label)
        target_form.addRow("Confidence:", self._conf_label)
        right.addWidget(target_group)

        # calculated z readout
        z_group = QGroupBox("Planned Z Values")
        z_form = QFormLayout(z_group)
        self._table_z_lbl  = QLabel("--")
        self._hover_z_lbl  = QLabel("--")
        self._raise_z_lbl  = QLabel("--")
        self._grab_z_lbl   = QLabel("--")
        self._lift_z_lbl   = QLabel("--")
        z_form.addRow("Table Z:",       self._table_z_lbl)
        z_form.addRow("Safe raise Z:",  self._raise_z_lbl)
        z_form.addRow("Hover Z:",       self._hover_z_lbl)
        z_form.addRow("Grab Z:",        self._grab_z_lbl)
        z_form.addRow("Lift Z:",        self._lift_z_lbl)
        right.addWidget(z_group)

        # motion toggle
        motion_group = QGroupBox("Motion")
        motion_layout = QVBoxLayout(motion_group)
        self._enable_motion_btn = QPushButton("Enable Motion on Pi Server")
        self._enable_motion_btn.setCheckable(True)
        self._enable_motion_btn.toggled.connect(self.enable_motion_requested.emit)
        motion_layout.addWidget(self._enable_motion_btn)
        self._motion_status = QLabel("Motion: disabled")
        motion_layout.addWidget(self._motion_status)
        right.addWidget(motion_group)

        # action buttons
        action_group = QGroupBox("Actions")
        action_layout = QVBoxLayout(action_group)
        self._hover_btn = QPushButton("Execute Hover Test")
        self._hover_btn.clicked.connect(self.hover_requested.emit)
        self._pick_btn = QPushButton("Execute Full Pickup")
        self._pick_btn.setObjectName("primaryButton")
        self._pick_btn.clicked.connect(self.pick_requested.emit)
        self._auto_pick_cb = QCheckBox("Auto-pick when object is stable")
        action_layout.addWidget(self._hover_btn)
        action_layout.addWidget(self._pick_btn)
        action_layout.addWidget(self._auto_pick_cb)
        right.addWidget(action_group)

        # pick status
        status_group = QGroupBox("Pick Status")
        status_layout = QVBoxLayout(status_group)
        self._pick_status = QLabel("Idle")
        self._pick_status.setWordWrap(True)
        status_layout.addWidget(self._pick_status)
        self._errors_label = QLabel("")
        self._errors_label.setWordWrap(True)
        self._errors_label.setStyleSheet("color: #f87171;")
        status_layout.addWidget(self._errors_label)
        right.addWidget(status_group)

        right.addStretch()
        layout.addLayout(right)

        # auto-pick state
        self._stable_since: float | None = None
        self._last_pick_at: float = 0.0
        self._current_detection: Detection | None = None
        self._calibrated = False
        self._motion_enabled = False

        self._auto_timer = QTimer(self)
        self._auto_timer.setInterval(200)
        self._auto_timer.timeout.connect(self._auto_tick)
        self._auto_timer.start()

    # ── public ───────────────────────────────────────────────────────────────

    def update_frame(self, frame: np.ndarray, det: Detection, robot_xy: tuple[float, float] | None) -> None:
        import cv2
        annotated = frame.copy()
        if det.found:
            if det.bounds:
                x, y, w, h = det.bounds
                cv2.rectangle(annotated, (x, y), (x + w, y + h), (80, 255, 80), 2)
            cv2.circle(annotated, (det.pixel_x, det.pixel_y), 6, (0, 0, 255), -1)
            label = f"conf={det.confidence:.2f}"
            if robot_xy:
                label += f"  robot=({robot_xy[0]:.1f}, {robot_xy[1]:.1f})"
            cv2.putText(annotated, label, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        h_px, w_px, ch = annotated.shape
        img = QImage(annotated.data, w_px, h_px, ch * w_px, QImage.Format.Format_BGR888)
        self._video.setPixmap(QPixmap.fromImage(img).scaled(
            640, 480,
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        ))

        self._current_detection = det
        if det.found:
            xy_text = f"({robot_xy[0]:.1f}, {robot_xy[1]:.1f}) mm" if robot_xy else "pixels only"
            self._det_label.setText(f"Object detected — {xy_text}")
            self._robot_xy_label.setText(xy_text)
            self._conf_label.setText(f"{det.confidence:.2f}")
            if self._stable_since is None:
                self._stable_since = time.monotonic()
        else:
            self._det_label.setText("No object detected")
            self._robot_xy_label.setText("--")
            self._conf_label.setText("--")
            self._stable_since = None

    def update_plan(self, plan: dict) -> None:
        calc = plan.get("calculated", {})
        def _fmt(v):
            return f"{v:.1f} mm" if isinstance(v, (int, float)) else "--"
        self._table_z_lbl.setText(_fmt(calc.get("tableZ")))
        self._raise_z_lbl.setText(_fmt(calc.get("safeRaiseZ")))
        self._hover_z_lbl.setText(_fmt(calc.get("hoverZ")))
        self._grab_z_lbl.setText( _fmt(calc.get("skimGrabZ")))
        self._lift_z_lbl.setText( _fmt(calc.get("liftZ")))

        errors = plan.get("errors", [])
        self._errors_label.setText("\n".join(errors) if errors else "")

    def set_pick_status(self, text: str) -> None:
        self._pick_status.setText(text)

    def set_button_state(self, *, calibrated: bool, pi_ok: bool, motion_enabled: bool) -> None:
        self._calibrated = calibrated
        self._motion_enabled = motion_enabled
        can_hover = calibrated and pi_ok
        can_pick = can_hover and motion_enabled
        self._hover_btn.setEnabled(can_hover)
        self._pick_btn.setEnabled(can_pick)
        self._auto_pick_cb.setEnabled(can_hover)
        self._motion_status.setText(f"Motion: {'enabled' if motion_enabled else 'disabled'}")

        # sync toggle without firing signal
        self._enable_motion_btn.blockSignals(True)
        self._enable_motion_btn.setChecked(motion_enabled)
        self._enable_motion_btn.blockSignals(False)

    # ── private ───────────────────────────────────────────────────────────────

    def _auto_tick(self) -> None:
        if not self._auto_pick_cb.isChecked():
            return
        det = self._current_detection
        if not (det and det.found and self._calibrated and self._motion_enabled):
            return
        now = time.monotonic()
        if self._stable_since is None:
            return
        stable_dur = now - self._stable_since
        since_last = now - self._last_pick_at
        if stable_dur >= _AUTO_STABLE_SEC and since_last >= _AUTO_COOLDOWN_SEC:
            self._last_pick_at = now
            self.pick_requested.emit()
