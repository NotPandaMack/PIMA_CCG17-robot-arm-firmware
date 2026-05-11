from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import QCheckBox, QFrame, QGridLayout, QLabel, QPushButton, QVBoxLayout, QWidget


class TestPage(QWidget):
    command_requested = Signal(str)
    jog_requested = Signal(str, int)
    refresh_requested = Signal()
    enable_motion_requested = Signal()
    dry_run_changed = Signal(bool)

    def __init__(self) -> None:
        super().__init__()
        layout = QGridLayout(self)
        layout.setSpacing(14)

        pose_card = QFrame()
        pose_card.setObjectName("panel")
        pose_layout = QVBoxLayout(pose_card)
        pose_layout.addWidget(QLabel("Manual Jog / Testing"))
        self.pose_label = QLabel("Current pose unavailable.")
        self.pose_label.setWordWrap(True)
        pose_layout.addWidget(self.pose_label)
        self.proposed_label = QLabel("Proposed next pose: none")
        self.proposed_label.setWordWrap(True)
        self.command_label = QLabel("Jog command: none")
        self.command_label.setWordWrap(True)
        pose_layout.addWidget(self.proposed_label)
        pose_layout.addWidget(self.command_label)
        refresh = QPushButton("Refresh Status")
        refresh.clicked.connect(self.refresh_requested.emit)
        pose_layout.addWidget(refresh)
        layout.addWidget(pose_card, 0, 0)

        jog_card = QFrame()
        jog_card.setObjectName("panel")
        jog_layout = QGridLayout(jog_card)
        self.adapter_dry_run = QCheckBox("Movement adapter dry-run")
        self.adapter_dry_run.setChecked(True)
        self.adapter_dry_run.setToolTip("Logs web-compatible commands without sending them to the ESP.")
        self.adapter_dry_run.toggled.connect(self.dry_run_changed.emit)
        jog_layout.addWidget(self.adapter_dry_run, 0, 0, 1, 2)
        jogs = [
            ("X-", "X", -1),
            ("X+", "X", 1),
            ("Y-", "Y", -1),
            ("Y+", "Y", 1),
            ("Z-", "Z", -1),
            ("Z+", "Z", 1),
            ("Pitch-", "PITCH", -1),
            ("Pitch+", "PITCH", 1),
        ]
        for index, (label, axis, direction) in enumerate(jogs):
            button = QPushButton(label)
            button.clicked.connect(lambda _checked=False, jog_axis=axis, jog_direction=direction: self.jog_requested.emit(jog_axis, jog_direction))
            jog_layout.addWidget(button, (index // 2) + 1, index % 2)
        self.calibration_lowering_mode = QCheckBox("Calibration lowering mode")
        self.calibration_lowering_mode.setToolTip("Allows small Z lowering during table touch calibration only.")
        jog_layout.addWidget(self.calibration_lowering_mode, 5, 0, 1, 2)
        warning = QLabel("Jogging sends full SET_TARGET commands using latest ESP X/Y/Z/Pitch. Z lowering is limited for table safety.")
        warning.setWordWrap(True)
        jog_layout.addWidget(warning, 6, 0, 1, 2)
        layout.addWidget(jog_card, 0, 1)

        actions = QFrame()
        actions.setObjectName("panel")
        actions_layout = QVBoxLayout(actions)
        for label, command in [
            ("Open Claw", "CLAW_OPEN"),
            ("Close Claw Soft", "CLAW_CLOSE_SOFT"),
            ("Stop", "STOP"),
            ("Clear Timeline", "CLEAR_TIMELINE"),
            ("ESTOP", "ESTOP"),
            ("Clear ESTOP", "CLEAR_ESTOP"),
        ]:
            button = QPushButton(label)
            if command == "ESTOP":
                button.setObjectName("dangerButton")
            button.clicked.connect(lambda _checked=False, cmd=command: self.command_requested.emit(cmd))
            actions_layout.addWidget(button)
        self.enable_motion_button = QPushButton("Enable Real Motion")
        self.enable_motion_button.setObjectName("dangerButton")
        self.enable_motion_button.clicked.connect(self.enable_motion_requested.emit)
        actions_layout.addWidget(self.enable_motion_button)
        actions_layout.addStretch(1)
        layout.addWidget(actions, 0, 2)
        layout.setColumnStretch(0, 2)
        layout.setColumnStretch(1, 1)
        layout.setColumnStretch(2, 1)

    def update_status(self, status: dict | None) -> None:
        if not status:
            self.pose_label.setText("Current pose unavailable.")
            return
        self.pose_label.setText(
            "\n".join(
                [
                    f"Status: {status.get('status', 'unknown')}",
                    f"X/Y/Z: {status.get('x', '--')} / {status.get('y', '--')} / {status.get('z', '--')}",
                    f"Pitch: {status.get('pitch', '--')}",
                    f"Claw: {status.get('clawTicks', '--')}",
                    f"ESTOP: {'active' if status.get('estop') else 'inactive'}",
                ]
            )
        )

    def set_motion_button_enabled(self, enabled: bool) -> None:
        self.enable_motion_button.setEnabled(enabled)

    def set_proposed_jog(self, pose: dict | None, command: str | None) -> None:
        if not pose or not command:
            self.proposed_label.setText("Proposed next pose: none")
            self.command_label.setText("Jog command: none")
            return
        self.proposed_label.setText(
            f"Proposed next pose: X {pose['x']:.1f}, Y {pose['y']:.1f}, Z {pose['z']:.1f}, Pitch {pose['pitch']:.1f}"
        )
        self.command_label.setText(f"Jog command: {command}")

    def calibration_touch_mode_enabled(self) -> bool:
        return self.calibration_lowering_mode.isChecked()

    def set_dry_run(self, enabled: bool) -> None:
        self.adapter_dry_run.setChecked(enabled)

    def dry_run_enabled(self) -> bool:
        return self.adapter_dry_run.isChecked()
