from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSpinBox,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from .camera_page import CameraView


class AutonomousPage(QWidget):
    preview_requested = Signal(bool)
    pick_requested = Signal(bool)
    auto_pick_changed = Signal(bool)

    def __init__(self) -> None:
        super().__init__()
        layout = QHBoxLayout(self)
        layout.setSpacing(14)

        left = QFrame()
        left.setObjectName("panel")
        left_layout = QVBoxLayout(left)
        self.camera_view = CameraView()
        left_layout.addWidget(QLabel("Autonomous Pickup"))
        left_layout.addWidget(self.camera_view, 1)
        self.target_label = QLabel("No target yet.")
        self.target_label.setWordWrap(True)
        left_layout.addWidget(self.target_label)
        layout.addWidget(left, 2)

        right = QFrame()
        right.setObjectName("panel")
        right_layout = QVBoxLayout(right)
        self.preview_button = QPushButton("Generate Preview")
        self.hover_preview_button = QPushButton("Send Hover Target to Website")
        self.hover_test_button = QPushButton("Hover Movement Disabled in PyGUI")
        self.pick_button = QPushButton("Full Pickup Disabled in PyGUI")
        self.pick_button.setObjectName("primaryButton")
        self.auto_pick = QCheckBox("Auto Pick When Object Is Stable")
        self.cooldown = QSpinBox()
        self.cooldown.setRange(2, 120)
        self.cooldown.setValue(8)
        self.plan_text = QTextEdit()
        self.plan_text.setReadOnly(True)
        for button in (self.preview_button, self.hover_preview_button, self.hover_test_button, self.pick_button):
            right_layout.addWidget(button)
        right_layout.addWidget(self.auto_pick)
        right_layout.addWidget(QLabel("Auto-pick cooldown seconds"))
        right_layout.addWidget(self.cooldown)
        right_layout.addWidget(QLabel("Website target / preview details"))
        right_layout.addWidget(self.plan_text, 1)
        layout.addWidget(right, 1)

        self.preview_button.clicked.connect(lambda: self.preview_requested.emit(False))
        self.hover_preview_button.clicked.connect(lambda: self.preview_requested.emit(True))
        self.hover_test_button.clicked.connect(lambda: self.pick_requested.emit(True))
        self.pick_button.clicked.connect(lambda: self.pick_requested.emit(False))
        self.auto_pick.toggled.connect(self.auto_pick_changed.emit)

    def update_target(self, text: str) -> None:
        self.target_label.setText(text)

    def update_plan(self, plan: dict) -> None:
        commands = plan.get("commands", [])
        calculated = plan.get("calculated", {})
        errors = plan.get("errors", [])
        lines = [
            f"ok: {plan.get('ok')}",
            f"motion enabled: {plan.get('motionEnabled')}",
            f"will send motion: {plan.get('willSendMotion')}",
            f"target robotX: {calculated.get('robotX')}",
            f"target robotY: {calculated.get('robotY')}",
            f"hoverZ: {calculated.get('hoverZ')}",
            f"skimGrabZ: {calculated.get('skimGrabZ')}",
            f"liftZ: {calculated.get('liftZ')}",
            f"pickupPitchDeg: {calculated.get('pickupPitchDeg')}",
            f"tableZ: {calculated.get('tableZ')}",
        ]
        if errors:
            lines.append("")
            lines.append("Blocked:")
            lines.extend(f"- {error}" for error in errors)
        lines.append("")
        lines.append("Commands:")
        lines.extend(commands or ["(no commands)"])
        self.plan_text.setPlainText("\n".join(lines))

    def set_buttons(self, *, can_preview: bool, can_hover: bool, can_pick: bool) -> None:
        self.preview_button.setEnabled(can_preview)
        self.hover_preview_button.setEnabled(can_preview)
        self.hover_test_button.setEnabled(can_hover)
        self.pick_button.setEnabled(can_pick)
