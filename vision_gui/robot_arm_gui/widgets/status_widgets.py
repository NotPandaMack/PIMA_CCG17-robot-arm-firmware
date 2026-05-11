from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QFrame, QHBoxLayout, QLabel, QVBoxLayout, QWidget

from ..calibration_manager import CHECKLIST_ITEMS, ChecklistState


class StatusPill(QLabel):
    def __init__(self, label: str = "", state: str = "red") -> None:
        super().__init__(label)
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumWidth(120)
        self.set_state(label, state)

    def set_state(self, label: str, state: str) -> None:
        self.setText(label)
        self.setProperty("state", state)
        self.style().unpolish(self)
        self.style().polish(self)


class SafetyBar(QFrame):
    def __init__(self) -> None:
        super().__init__()
        self.setObjectName("safetyBar")
        layout = QHBoxLayout(self)
        layout.setContentsMargins(16, 10, 16, 10)
        layout.setSpacing(10)

        self.motion = StatusPill("GHOST MODE", "yellow")
        self.estop = StatusPill("ESTOP unknown", "yellow")
        self.calibration = StatusPill("Calibration unknown", "yellow")
        self.target = StatusPill("Target none", "yellow")

        title = QLabel("Robot Arm Control Center")
        title.setObjectName("appTitle")
        layout.addWidget(title, 1)
        layout.addWidget(self.motion)
        layout.addWidget(self.estop)
        layout.addWidget(self.calibration)
        layout.addWidget(self.target)

    def update_state(self, *, motion_enabled: bool, estop_active: bool | None, calibrated: bool, target_status: str) -> None:
        self.motion.set_state("MOTION ENABLED" if motion_enabled else "GHOST MODE", "green" if motion_enabled else "yellow")
        if estop_active is None:
            self.estop.set_state("ESTOP unknown", "yellow")
        else:
            self.estop.set_state("ESTOP ACTIVE" if estop_active else "ESTOP clear", "red" if estop_active else "green")
        self.calibration.set_state("Calibrated" if calibrated else "Calibration needed", "green" if calibrated else "red")
        target_state = "green" if target_status == "valid" else "yellow" if target_status in {"none", "pixels"} else "red"
        self.target.set_state(f"Target {target_status}", target_state)


class ChecklistPanel(QFrame):
    def __init__(self) -> None:
        super().__init__()
        self.setObjectName("checklistPanel")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)
        title = QLabel("Beginner Setup Checklist")
        title.setObjectName("sectionTitle")
        layout.addWidget(title)
        self.labels: dict[str, QLabel] = {}
        for key, label in CHECKLIST_ITEMS:
            row = QLabel()
            row.setObjectName("checklistItem")
            self.labels[key] = row
            layout.addWidget(row)
        layout.addStretch(1)

    def update_state(self, state: ChecklistState) -> None:
        values = state.as_dict()
        for key, label in CHECKLIST_ITEMS:
            complete = bool(values.get(key))
            widget = self.labels[key]
            widget.setText(f"{'[x]' if complete else '[ ]'} {label}")
            widget.setProperty("complete", complete)
            widget.style().unpolish(widget)
            widget.style().polish(widget)
