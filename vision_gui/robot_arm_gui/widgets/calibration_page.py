from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QDoubleSpinBox,
    QFormLayout,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QProgressBar,
    QSpinBox,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from ..config import DEFAULT_WORKSPACE
from .camera_page import CameraView


CALIBRATION_STEPS = [
    ("Camera Placement", "Mount your webcam so the full reachable table area, robot base, and all four marker locations are visible."),
    ("Define Robot Origin", "Click the robot base center projected onto the table. This becomes robot coordinate X=0, Y=0."),
    ("Four-Point Table Mapping", "Place each marker, enter its real robot/table X/Y coordinate in millimeters, then click it in the camera image."),
    ("Workspace Bounds", "These safety limits stop the robot from accepting targets outside the reachable box."),
    ("Table Z Calibration", "Manually jog the claw/tip until it barely touches the table, then save the current ESP pose. The GUI never lowers the arm automatically."),
    ("Pickup Pose", "Manually move the arm to a good pickup pose near the table, then save the current ESP pose and claw values."),
    ("Calibration Validation", "Click a table point. The app converts it to robot coordinates and can generate a hover preview."),
    ("Finish Calibration", "Save calibration locally and upload it to the Pi server."),
]

MAPPING_DEFAULTS = [
    ("front-left", -150.0, 100.0),
    ("front-right", 150.0, 100.0),
    ("back-left", -150.0, 260.0),
    ("back-right", 150.0, 260.0),
]

TABLE_POINTS = ["center-near", "center-far", "left-mid", "right-mid", "optional-center"]


class CalibrationPage(QWidget):
    step_changed = Signal(int)
    save_step_requested = Signal()
    save_touch_requested = Signal(str)
    save_pickup_requested = Signal()
    preview_hover_requested = Signal()
    test_hover_requested = Signal()
    finish_requested = Signal()

    def __init__(self) -> None:
        super().__init__()
        self.step_index = 0
        self.origin_pixel: tuple[int, int] | None = None
        self.mapping_pixels: dict[str, tuple[int, int]] = {}
        self.table_points: list[dict] = []
        self.validation_pixel: tuple[int, int] | None = None

        layout = QGridLayout(self)
        layout.setSpacing(14)

        left = QFrame()
        left.setObjectName("panel")
        left_layout = QVBoxLayout(left)
        self.title = QLabel()
        self.title.setObjectName("sectionTitle")
        self.instructions = QLabel()
        self.instructions.setWordWrap(True)
        self.progress = QProgressBar()
        self.progress.setRange(0, len(CALIBRATION_STEPS))
        self.camera_view = CameraView()
        left_layout.addWidget(self.title)
        left_layout.addWidget(self.instructions)
        left_layout.addWidget(self.progress)
        left_layout.addWidget(self.camera_view, 1)
        layout.addWidget(left, 0, 0)

        right = QFrame()
        right.setObjectName("panel")
        right_layout = QVBoxLayout(right)
        self.stack = QStackedWidget()
        self.stack.addWidget(self._camera_step())
        self.stack.addWidget(self._origin_step())
        self.stack.addWidget(self._mapping_step())
        self.stack.addWidget(self._workspace_step())
        self.stack.addWidget(self._table_z_step())
        self.stack.addWidget(self._pickup_step())
        self.stack.addWidget(self._validation_step())
        self.stack.addWidget(self._finish_step())
        right_layout.addWidget(self.stack, 1)

        nav = QHBoxLayout()
        self.back_button = QPushButton("Back")
        self.save_step_button = QPushButton("Save Step")
        self.next_button = QPushButton("Next")
        self.next_button.setObjectName("primaryButton")
        nav.addWidget(self.back_button)
        nav.addWidget(self.save_step_button)
        nav.addWidget(self.next_button)
        right_layout.addLayout(nav)
        layout.addWidget(right, 0, 1)
        layout.setColumnStretch(0, 3)
        layout.setColumnStretch(1, 2)

        self.back_button.clicked.connect(lambda: self.set_step(max(0, self.step_index - 1)))
        self.next_button.clicked.connect(lambda: self.set_step(min(len(CALIBRATION_STEPS) - 1, self.step_index + 1)))
        self.save_step_button.clicked.connect(self.save_step_requested.emit)
        self.set_step(0)

    def set_step(self, index: int) -> None:
        self.step_index = int(index)
        title, instructions = CALIBRATION_STEPS[self.step_index]
        self.title.setText(title)
        self.instructions.setText(instructions)
        self.stack.setCurrentIndex(self.step_index)
        self.progress.setValue(self.step_index + 1)
        self.back_button.setEnabled(self.step_index > 0)
        self.next_button.setEnabled(self.step_index < len(CALIBRATION_STEPS) - 1)
        self.step_changed.emit(self.step_index)

    def capture_click(self, x: int, y: int) -> None:
        if self.step_index == 1:
            self.origin_pixel = (x, y)
            self.origin_label.setText(f"Origin pixel: {x}, {y}")
        elif self.step_index == 2:
            label = self.current_marker_label()
            self.mapping_pixels[label] = (x, y)
            self.marker_pixel_labels[label].setText(f"{x}, {y}")
        elif self.step_index == 6:
            self.validation_pixel = (x, y)
            self.validation_label.setText(f"Clicked pixel: {x}, {y}")

    def current_marker_label(self) -> str:
        return MAPPING_DEFAULTS[self.current_marker.value()][0]

    def mapping_points(self) -> list[dict]:
        points = []
        for label, _, _ in MAPPING_DEFAULTS:
            if label not in self.mapping_pixels:
                continue
            px, py = self.mapping_pixels[label]
            points.append(
                {
                    "label": label,
                    "pixel": {"x": px, "y": py},
                    "robot": {"x": self.robot_x_inputs[label].value(), "y": self.robot_y_inputs[label].value()},
                }
            )
        return points

    def workspace(self) -> dict[str, float]:
        return {key: widget.value() for key, widget in self.workspace_inputs.items()}

    def pickup_values(self) -> dict:
        return {
            "pickupPitchDeg": self.pickup_pitch.value(),
            "skimZ": self.skim_z.value(),
            "grabOffsetZ": self.grab_offset_z.value(),
            "hoverZ": self.hover_z.value(),
            "liftZ": self.lift_z.value(),
            "clawOpenValue": self.claw_open.value(),
            "clawClosedValue": self.claw_closed.value(),
        }

    def set_pickup_from_status(self, status: dict) -> None:
        self.pickup_pitch.setValue(float(status.get("pitch", self.pickup_pitch.value())))
        self.skim_z.setValue(float(status.get("z", self.skim_z.value())))
        self.claw_closed.setValue(float(status.get("clawTicks", self.claw_closed.value())))

    def add_table_point(self, label: str, status: dict) -> None:
        point = {
            "label": label,
            "x": float(status.get("x", 0.0)),
            "y": float(status.get("y", 0.0)),
            "z": float(status.get("z", 0.0)),
        }
        self.table_points = [existing for existing in self.table_points if existing["label"] != label]
        self.table_points.append(point)
        self.table_status.setText("\n".join(f"{p['label']}: X {p['x']:.1f}, Y {p['y']:.1f}, Z {p['z']:.1f}" for p in self.table_points))

    def table_z(self) -> dict:
        if len(self.table_points) >= 3:
            return {"method": "plane", "points": self.table_points}
        return {"method": "placeholder", "z": 0.0, "points": self.table_points}

    def set_validation_result(self, text: str) -> None:
        self.validation_result.setText(text)

    def _camera_step(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        self.camera_status_label = QLabel("Start the camera and confirm the table is visible.")
        self.camera_status_label.setWordWrap(True)
        layout.addWidget(self.camera_status_label)
        layout.addStretch(1)
        return page

    def _origin_step(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        self.origin_label = QLabel("Origin pixel: not selected")
        layout.addWidget(self.origin_label)
        layout.addStretch(1)
        return page

    def _mapping_step(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        self.current_marker = QSpinBox()
        self.current_marker.setRange(0, len(MAPPING_DEFAULTS) - 1)
        self.current_marker.setPrefix("Marker ")
        layout.addWidget(self.current_marker)
        self.marker_pixel_labels: dict[str, QLabel] = {}
        self.robot_x_inputs: dict[str, QDoubleSpinBox] = {}
        self.robot_y_inputs: dict[str, QDoubleSpinBox] = {}
        for label, default_x, default_y in MAPPING_DEFAULTS:
            box = QFrame()
            box.setObjectName("subPanel")
            form = QFormLayout(box)
            pixel_label = QLabel("not clicked")
            robot_x = self._double(default_x, -500.0, 500.0)
            robot_y = self._double(default_y, -100.0, 500.0)
            self.marker_pixel_labels[label] = pixel_label
            self.robot_x_inputs[label] = robot_x
            self.robot_y_inputs[label] = robot_y
            form.addRow(QLabel(label))
            form.addRow("Pixel", pixel_label)
            form.addRow("Robot X mm", robot_x)
            form.addRow("Robot Y mm", robot_y)
            layout.addWidget(box)
        layout.addStretch(1)
        return page

    def _workspace_step(self) -> QWidget:
        page = QWidget()
        form = QFormLayout(page)
        self.workspace_inputs = {}
        for key, value in DEFAULT_WORKSPACE.items():
            widget = self._double(float(value), -1000.0, 1000.0)
            self.workspace_inputs[key] = widget
            form.addRow(key, widget)
        return page

    def _table_z_step(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        self.table_point_index = QSpinBox()
        self.table_point_index.setRange(0, len(TABLE_POINTS) - 1)
        layout.addWidget(self.table_point_index)
        button = QPushButton("Save Touch Point From Current ESP Pose")
        button.setObjectName("primaryButton")
        button.clicked.connect(lambda: self.save_touch_requested.emit(TABLE_POINTS[self.table_point_index.value()]))
        self.table_status = QLabel("No touch points saved.")
        self.table_status.setWordWrap(True)
        layout.addWidget(button)
        layout.addWidget(self.table_status)
        layout.addStretch(1)
        return page

    def _pickup_step(self) -> QWidget:
        page = QWidget()
        form = QFormLayout(page)
        self.pickup_pitch = self._double(-8.0, -90.0, 90.0)
        self.skim_z = self._double(10.0, -100.0, 250.0)
        self.grab_offset_z = self._double(10.0, -100.0, 250.0)
        self.hover_z = self._double(80.0, 0.0, 250.0)
        self.lift_z = self._double(100.0, 0.0, 250.0)
        self.claw_open = self._int(0, 0, 180)
        self.claw_closed = self._int(55, 0, 180)
        form.addRow("Pickup pitch deg", self.pickup_pitch)
        form.addRow("Skim Z", self.skim_z)
        form.addRow("Grab offset Z", self.grab_offset_z)
        form.addRow("Hover Z", self.hover_z)
        form.addRow("Lift Z", self.lift_z)
        form.addRow("Claw open value", self.claw_open)
        form.addRow("Claw closed value", self.claw_closed)
        button = QPushButton("Save Pickup Pose From ESP")
        button.clicked.connect(self.save_pickup_requested.emit)
        form.addRow(button)
        return page

    def _validation_step(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        self.validation_label = QLabel("Clicked pixel: none")
        self.validation_result = QLabel("Conversion result: none")
        self.validation_result.setWordWrap(True)
        preview = QPushButton("Preview Hover")
        test = QPushButton("Test Hover")
        preview.clicked.connect(self.preview_hover_requested.emit)
        test.clicked.connect(self.test_hover_requested.emit)
        layout.addWidget(self.validation_label)
        layout.addWidget(self.validation_result)
        layout.addWidget(preview)
        layout.addWidget(test)
        layout.addStretch(1)
        return page

    def _finish_step(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        self.finish_status = QLabel("Save calibration when every required item is complete.")
        self.finish_status.setWordWrap(True)
        button = QPushButton("Finish Calibration")
        button.setObjectName("primaryButton")
        button.clicked.connect(self.finish_requested.emit)
        layout.addWidget(self.finish_status)
        layout.addWidget(button)
        layout.addStretch(1)
        return page

    @staticmethod
    def _double(value: float, minimum: float, maximum: float) -> QDoubleSpinBox:
        widget = QDoubleSpinBox()
        widget.setRange(minimum, maximum)
        widget.setDecimals(1)
        widget.setSingleStep(1.0)
        widget.setValue(value)
        return widget

    @staticmethod
    def _int(value: int, minimum: int, maximum: int) -> QSpinBox:
        widget = QSpinBox()
        widget.setRange(minimum, maximum)
        widget.setValue(value)
        return widget
