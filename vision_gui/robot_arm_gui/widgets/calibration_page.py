from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QProgressBar,
    QSpinBox,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from ..calibration_markers import MARKER_DEFINITIONS, mapping_warnings
from ..calibration_manager import fit_side_view_table_z
from ..config import DEFAULT_WORKSPACE
from .camera_page import CameraView


CALIBRATION_STEPS = [
    ("Camera Placement", "Mount your webcam so the full reachable table area, robot base, and all four marker locations are visible."),
    ("Define Robot Origin", "Click the robot base center projected onto the table. This becomes robot coordinate X=0, Y=0."),
    ("Four-Point Table Mapping", "Place each marker, enter its real robot/table X/Y coordinate in millimeters, then click it in the camera image."),
    ("Workspace Bounds", "These safety limits stop the robot from accepting targets outside the reachable box."),
    ("Side-View Table Z Calibration", "Display the monitor board full-screen behind the robot, start the iPhone side camera, set the table line, then save safe visual claw-height samples. The GUI never moves or lowers the arm."),
    ("Pickup Pose", "Use the website 2D IK simulator or Specific Joint Adjustment to make a good pickup pose, then save the current control/ESP pose and claw values."),
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
    start_side_camera_requested = Signal(str)
    stop_side_camera_requested = Signal()
    detect_side_board_requested = Signal()
    save_side_sample_requested = Signal()
    save_pickup_requested = Signal()
    preview_hover_requested = Signal()
    test_hover_requested = Signal()
    finish_requested = Signal()
    grid_overlay_changed = Signal(bool)
    generate_aruco_requested = Signal()
    scan_aruco_requested = Signal()
    generate_qr_requested = Signal()
    scan_qr_requested = Signal()
    marker_overlay_changed = Signal(object)

    def __init__(self) -> None:
        super().__init__()
        self.step_index = 0
        self.origin_pixel: tuple[int, int] | None = None
        self.mapping_pixels: dict[str, tuple[int, int]] = {}
        self.mapping_sources: dict[str, str] = {}
        self.mapping_corners: dict[str, list[dict]] = {}
        self.table_points: list[dict] = []
        self.side_samples: list[dict] = []
        self.side_table_line: dict | None = None
        self.side_pending_tip: tuple[int, int] | None = None
        self.side_board_markers: list[dict] = []
        self.side_fit: dict | None = None
        self._side_table_clicks: list[tuple[int, int]] = []
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
        self.grid_overlay = QCheckBox("Show calibration grid overlay")
        self.camera_view = CameraView()
        left_layout.addWidget(self.title)
        left_layout.addWidget(self.instructions)
        left_layout.addWidget(self.progress)
        left_layout.addWidget(self.grid_overlay)
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
        self.grid_overlay.toggled.connect(self.grid_overlay_changed.emit)
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

    def capture_click(self, x: int, y: int, debug: dict | None = None) -> None:
        if self.step_index == 1:
            self.origin_pixel = (x, y)
            self.origin_label.setText(f"Origin pixel: {x}, {y}")
            self._emit_marker_overlay_changed()
        elif self.step_index == 2:
            if not self.manual_fallback.isChecked():
                self.mapping_help.setText("Manual clicks are locked. Enable manual click fallback to place markers by clicking.")
                return
            label = self.current_marker_label()
            self.set_mapping_marker(label, x, y, source="manual")
        elif self.step_index == 6:
            self.validation_pixel = (x, y)
            if debug:
                self.validation_label.setText(
                    f"Widget click: {debug.get('widgetX', 0):.1f}, {debug.get('widgetY', 0):.1f}\n"
                    f"Corrected image pixel: {x}, {y}\n"
                    f"Displayed-image position: {debug.get('displayX', 0):.1f}, {debug.get('displayY', 0):.1f}"
                )
            else:
                self.validation_label.setText(f"Clicked pixel: {x}, {y}")

    def current_marker_label(self) -> str:
        return str(self.current_marker.currentData() or MAPPING_DEFAULTS[0][0])

    def set_mapping_marker(
        self,
        label: str,
        pixel_x: float,
        pixel_y: float,
        *,
        source: str,
        robot_x: float | None = None,
        robot_y: float | None = None,
        corners: list[dict] | None = None,
    ) -> None:
        x = int(round(pixel_x))
        y = int(round(pixel_y))
        self.mapping_pixels[label] = (x, y)
        self.mapping_sources[label] = source
        if corners:
            self.mapping_corners[label] = corners
        elif label in self.mapping_corners:
            del self.mapping_corners[label]
        if robot_x is not None:
            self.robot_x_inputs[label].setValue(float(robot_x))
        if robot_y is not None:
            self.robot_y_inputs[label].setValue(float(robot_y))
        self.marker_pixel_labels[label].setText(f"{x}, {y}")
        self.marker_source_labels[label].setText(source)
        self.marker_check_labels[label].setText("OK")
        self._update_mapping_summary()
        self._emit_marker_overlay_changed()

    def apply_detected_markers(self, markers: list[dict], warnings: list[str] | None = None) -> None:
        for marker in markers:
            label = marker.get("label")
            pixel = marker.get("pixel", {})
            robot = marker.get("robot", {})
            if label not in self.robot_x_inputs:
                continue
            self.set_mapping_marker(
                label,
                float(pixel.get("x", 0.0)),
                float(pixel.get("y", 0.0)),
                source=str(marker.get("source", "aruco")),
                robot_x=float(robot.get("x", self.robot_x_inputs[label].value())),
                robot_y=float(robot.get("y", self.robot_y_inputs[label].value())),
                corners=marker.get("corners") if isinstance(marker.get("corners"), list) else None,
            )
        self._update_mapping_summary(warnings or [])
        self._emit_marker_overlay_changed()

    def set_mapping_detection_status(self, text: str) -> None:
        self.mapping_help.setText(text)

    def set_mapping_quality(self, text: str) -> None:
        self.mapping_quality.setText(text)

    def set_aruco_status(self, text: str, available: bool) -> None:
        self.aruco_status.setText(text)
        self.scan_aruco_button.setEnabled(available)
        self.generate_aruco_button.setEnabled(available)

    def mapping_overlay(self) -> dict[str, dict]:
        markers = {}
        for label, (x, y) in self.mapping_pixels.items():
            short = _short_label(label)
            markers[label] = {
                "label": label,
                "short": short,
                "pixel": {"x": x, "y": y},
                "robot": {"x": self.robot_x_inputs[label].value(), "y": self.robot_y_inputs[label].value()},
                "source": self.mapping_sources.get(label, "manual"),
                "corners": self.mapping_corners.get(label),
            }
        return markers

    def _update_mapping_summary(self, warnings: list[str] | None = None) -> None:
        missing = [_short_label(label) for label, _x, _y in MAPPING_DEFAULTS if label not in self.mapping_pixels]
        all_warnings = list(warnings or [])
        if len(self.mapping_pixels) >= 2:
            all_warnings.extend(mapping_warnings(list(self.mapping_overlay().values())))
        if missing:
            text = f"Need markers: {', '.join(missing)}."
        else:
            text = "Ready to compute homography."
        if all_warnings:
            text += "\nWarnings: " + " ".join(dict.fromkeys(all_warnings))
        self.mapping_quality.setText(text)

    def _emit_marker_overlay_changed(self) -> None:
        if hasattr(self, "mapping_quality"):
            self._update_mapping_summary()
        self.marker_overlay_changed.emit(self.mapping_overlay())

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
                    "source": self.mapping_sources.get(label, "manual"),
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

    def set_pickup_source_status(self, text: str) -> None:
        self.pickup_source_status.setText(text)

    def add_table_point(self, label: str, status: dict) -> None:
        point = {
            "label": label,
            "x": float(status.get("x", 0.0)),
            "y": float(status.get("y", 0.0)),
            "z": float(status.get("z", 0.0)),
        }
        if status.get("pitch") is not None:
            point["pitch"] = float(status["pitch"])
        if status.get("source"):
            point["source"] = status["source"]
        if status.get("manualControlState"):
            point["manualControlState"] = status["manualControlState"]
        if status.get("espStatus"):
            point["espStatus"] = status["espStatus"]
        self.table_points = [existing for existing in self.table_points if existing["label"] != label]
        self.table_points.append(point)
        if hasattr(self, "table_point_status_labels") and label in self.table_point_status_labels:
            self.table_point_status_labels[label].setText(f"Saved X {point['x']:.1f}, Y {point['y']:.1f}, Z {point['z']:.1f}")
            self.table_point_source_labels[label].setText(str(status.get("source", "unknown source")))
        self.table_status.setText("\n".join(f"{p['label']}: X {p['x']:.1f}, Y {p['y']:.1f}, Z {p['z']:.1f}" for p in self.table_points))
        method = "plane" if len(self.table_points) >= 3 else "placeholder"
        self.table_z_summary.setText(f"Table Z method: {method}. Saved points: {len(self.table_points)} / 4 required plus optional center.")
        self.table_source_status.setText(f"Saved {label} from {status.get('source', 'unknown source')}.")

    def handle_side_click(self, x: int, y: int) -> None:
        if self.side_click_mode.currentData() == "table":
            self._side_table_clicks.append((x, y))
            if len(self._side_table_clicks) >= 2:
                p1, p2 = self._side_table_clicks[-2], self._side_table_clicks[-1]
                self.side_table_line = {"p1": {"x": p1[0], "y": p1[1]}, "p2": {"x": p2[0], "y": p2[1]}}
                self.side_table_status.setText(f"Table line set: ({p1[0]}, {p1[1]}) to ({p2[0]}, {p2[1]}).")
                self._side_table_clicks = []
                self._update_side_fit()
                self._update_side_overlay()
            else:
                self.side_table_status.setText(f"Table line first point: {x}, {y}. Click the second point.")
            return

        self.side_pending_tip = (x, y)
        self.side_tip_status.setText(f"Pending claw-tip click: {x}, {y}. Press Save Side Sample after checking the current Z.")
        self._update_side_overlay()

    def add_side_sample(self, status: dict) -> None:
        if self.side_pending_tip is None:
            raise ValueError("click the visible claw tip in the side-view image before saving")
        if not isinstance(status.get("z"), (int, float)):
            raise ValueError("no valid ESP pose or website IK draft Z is available")
        x, y = self.side_pending_tip
        sample = {
            "robotZ": float(status["z"]),
            "pixel": {"x": int(x), "y": int(y)},
            "source": status.get("source", "unknown source"),
        }
        if status.get("manualControlState"):
            sample["manualControlState"] = status["manualControlState"]
        if status.get("espStatus"):
            sample["espStatus"] = status["espStatus"]
        self.side_samples.append(sample)
        self.side_pending_tip = None
        self.side_tip_status.setText("Pending claw-tip click: none.")
        self._update_side_fit()
        self._update_side_sample_cards()
        self._update_side_overlay()

    def apply_side_board_detection(self, result: dict) -> None:
        self.side_board_markers = result.get("markers", []) if isinstance(result.get("markers"), list) else []
        warnings = result.get("warnings", []) if isinstance(result.get("warnings"), list) else []
        warning_text = f" Warnings: {' '.join(warnings)}" if warnings else ""
        self.side_board_status.setText(f"Monitor board markers detected: {len(self.side_board_markers)}.{warning_text}")
        self._update_side_overlay()

    def set_side_camera_status(self, text: str) -> None:
        self.side_camera_status.setText(text)

    def set_side_camera_url(self, stream_url: str) -> None:
        if hasattr(self, "side_stream_url") and not self.side_stream_url.text().strip():
            self.side_stream_url.setText(stream_url)

    def table_z(self) -> dict:
        if self.side_fit and self.side_fit.get("method") == "side_view_visual_fit":
            return self.side_fit
        if len(self.table_points) >= 3:
            return {"method": "plane", "points": self.table_points}
        return {"method": "placeholder", "z": 0.0, "points": self.table_points}

    def _update_side_fit(self) -> None:
        if not self.side_table_line or len(self.side_samples) < 2:
            self.side_fit = None
            self.side_fit_status.setText(f"Fit ready: no. Samples: {len(self.side_samples)} / 2 minimum.")
            return
        try:
            self.side_fit = fit_side_view_table_z(
                samples=self.side_samples,
                table_line=self.side_table_line,
                safety_margin_mm=self.side_safety_margin.value(),
            )
        except Exception as error:
            self.side_fit = None
            self.side_fit_status.setText(f"Fit ready: no. {error}")
            return
        fit = self.side_fit.get("fit", {})
        self.side_fit_status.setText(
            f"Fit ready: yes. visual tableZ {self.side_fit['z']:.1f} mm, "
            f"safety margin {self.side_fit['safetyMarginMm']:.1f} mm, "
            f"{fit.get('pixelsPerRobotMm', 0.0):.2f} px/mm, error {fit.get('errorPx', 0.0):.1f} px."
        )

    def _update_side_sample_cards(self) -> None:
        high = max((sample["robotZ"] for sample in self.side_samples), default=None)
        low = min((sample["robotZ"] for sample in self.side_samples), default=None)
        self.side_high_status.setText("High sample saved" if high is not None else "High sample missing")
        self.side_low_status.setText("Low sample saved" if low is not None and high is not None and low < high else "Low sample missing")
        self.side_samples_status.setText(
            "\n".join(
                f"S{index}: Z {sample['robotZ']:.1f}, px {sample['pixel']['x']}, {sample['pixel']['y']} ({sample.get('source', 'unknown')})"
                for index, sample in enumerate(self.side_samples, start=1)
            )
            or "No side-view samples saved."
        )

    def _update_side_overlay(self) -> None:
        if hasattr(self, "side_camera_view"):
            samples = list(self.side_samples)
            if self.side_pending_tip:
                samples.append({"robotZ": 0.0, "pixel": {"x": self.side_pending_tip[0], "y": self.side_pending_tip[1]}, "source": "pending"})
            self.side_camera_view.set_side_overlay(
                table_line=self.side_table_line,
                samples=samples,
                board_markers=self.side_board_markers,
                fit=self.side_fit,
            )

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
        self.aruco_status = QLabel("ArUco is the primary path. Support is checked when you generate or scan markers.")
        self.aruco_status.setWordWrap(True)
        layout.addWidget(self.aruco_status)

        action_row = QHBoxLayout()
        self.generate_aruco_button = QPushButton("Generate ArUco Marker Sheet")
        self.scan_aruco_button = QPushButton("Scan ArUco Markers From Camera")
        self.generate_qr_button = QPushButton("Generate QR Fallback Sheet")
        self.scan_qr_button = QPushButton("Scan QR Fallback")
        for button in (self.generate_aruco_button, self.scan_aruco_button, self.generate_qr_button, self.scan_qr_button):
            action_row.addWidget(button)
        layout.addLayout(action_row)

        self.mapping_help = QLabel("Place printed ArUco markers at FL/FR/BL/BR, then scan from the live camera.")
        self.mapping_help.setWordWrap(True)
        layout.addWidget(self.mapping_help)

        self.manual_fallback = QCheckBox("Use manual click fallback")
        layout.addWidget(self.manual_fallback)

        self.current_marker = QComboBox()
        for label, _default_x, _default_y in MAPPING_DEFAULTS:
            self.current_marker.addItem(f"Manual marker: {_short_label(label)} - {label}", label)
        self.current_marker.setEnabled(False)
        layout.addWidget(self.current_marker)

        self.marker_pixel_labels: dict[str, QLabel] = {}
        self.marker_source_labels: dict[str, QLabel] = {}
        self.marker_check_labels: dict[str, QLabel] = {}
        self.robot_x_inputs: dict[str, QDoubleSpinBox] = {}
        self.robot_y_inputs: dict[str, QDoubleSpinBox] = {}
        for label, default_x, default_y in MAPPING_DEFAULTS:
            box = QFrame()
            box.setObjectName("subPanel")
            form = QFormLayout(box)
            status = QLabel("missing")
            pixel_label = QLabel("not clicked")
            source_label = QLabel("none")
            robot_x = self._double(default_x, -500.0, 500.0)
            robot_y = self._double(default_y, -100.0, 500.0)
            robot_x.valueChanged.connect(lambda _value, self=self: self._emit_marker_overlay_changed())
            robot_y.valueChanged.connect(lambda _value, self=self: self._emit_marker_overlay_changed())
            self.marker_pixel_labels[label] = pixel_label
            self.marker_source_labels[label] = source_label
            self.marker_check_labels[label] = status
            self.robot_x_inputs[label] = robot_x
            self.robot_y_inputs[label] = robot_y
            form.addRow(QLabel(f"{_short_label(label)} - {label}"))
            form.addRow("Status", status)
            form.addRow("Pixel", pixel_label)
            form.addRow("Source", source_label)
            form.addRow("Robot X mm", robot_x)
            form.addRow("Robot Y mm", robot_y)
            layout.addWidget(box)
        self.mapping_quality = QLabel("Need FL, FR, BL, BR markers.")
        self.mapping_quality.setWordWrap(True)
        layout.addWidget(self.mapping_quality)
        self.generate_aruco_button.clicked.connect(self.generate_aruco_requested.emit)
        self.scan_aruco_button.clicked.connect(self.scan_aruco_requested.emit)
        self.generate_qr_button.clicked.connect(self.generate_qr_requested.emit)
        self.scan_qr_button.clicked.connect(self.scan_qr_requested.emit)
        self.manual_fallback.toggled.connect(self.current_marker.setEnabled)
        self.manual_fallback.toggled.connect(lambda _checked: self._emit_marker_overlay_changed())
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
        intro = QLabel("Open the website on the monitor as ?mode=side-calibration-board and put it full screen behind the robot. Move only from the trusted website controls, then record visual claw-tip samples here.")
        intro.setWordWrap(True)
        layout.addWidget(intro)

        stream_box = QFrame()
        stream_box.setObjectName("subPanel")
        stream_layout = QVBoxLayout(stream_box)
        self.side_stream_url = QLineEdit()
        self.side_stream_url.setPlaceholderText("http://iphone-ip:port/video or rtsp://...")
        button_row = QHBoxLayout()
        start_button = QPushButton("Start Side Camera")
        stop_button = QPushButton("Stop Side Camera")
        detect_button = QPushButton("Detect Board")
        start_button.clicked.connect(lambda: self.start_side_camera_requested.emit(self.side_stream_url.text().strip()))
        stop_button.clicked.connect(self.stop_side_camera_requested.emit)
        detect_button.clicked.connect(self.detect_side_board_requested.emit)
        button_row.addWidget(start_button)
        button_row.addWidget(stop_button)
        button_row.addWidget(detect_button)
        self.side_camera_status = QLabel("Side camera stopped.")
        self.side_camera_status.setWordWrap(True)
        stream_layout.addWidget(QLabel("Side View / iPhone Camera"))
        stream_layout.addWidget(self.side_stream_url)
        stream_layout.addLayout(button_row)
        stream_layout.addWidget(self.side_camera_status)
        layout.addWidget(stream_box)

        self.side_camera_view = CameraView()
        self.side_camera_view.setMinimumSize(480, 270)
        self.side_camera_view.mapped_clicked.connect(lambda mapped: self.handle_side_click(int(mapped["pixelX"]), int(mapped["pixelY"])))
        layout.addWidget(self.side_camera_view)

        controls = QFrame()
        controls.setObjectName("subPanel")
        controls_layout = QFormLayout(controls)
        self.side_click_mode = QComboBox()
        self.side_click_mode.addItem("Click table line points", "table")
        self.side_click_mode.addItem("Click claw tip sample", "tip")
        self.side_safety_margin = self._double(8.0, 0.0, 50.0)
        controls_layout.addRow("Side-view click mode", self.side_click_mode)
        controls_layout.addRow("Safety margin mm", self.side_safety_margin)
        layout.addWidget(controls)

        self.side_board_status = QLabel("Monitor board detected: no.")
        self.side_table_status = QLabel("Table line set: no.")
        self.side_tip_status = QLabel("Pending claw-tip click: none.")
        self.side_high_status = QLabel("High sample missing")
        self.side_low_status = QLabel("Low sample missing")
        self.side_fit_status = QLabel("Fit ready: no. Samples: 0 / 2 minimum.")
        for label in (
            self.side_board_status,
            self.side_table_status,
            self.side_tip_status,
            self.side_high_status,
            self.side_low_status,
            self.side_fit_status,
        ):
            label.setWordWrap(True)
            layout.addWidget(label)

        save_side_sample = QPushButton("Save Side Sample From Current ESP Pose")
        save_side_sample.setObjectName("primaryButton")
        save_side_sample.clicked.connect(self.save_side_sample_requested.emit)
        layout.addWidget(save_side_sample)
        self.side_samples_status = QLabel("No side-view samples saved.")
        self.side_samples_status.setWordWrap(True)
        layout.addWidget(self.side_samples_status)

        fallback_label = QLabel("Advanced manual touch fallback")
        fallback_label.setObjectName("sectionTitle")
        layout.addWidget(fallback_label)
        fallback_intro = QLabel("Only use this if visual side-view calibration is unavailable. Move the arm from the website and save current poses; this GUI still never sends lowering commands.")
        fallback_intro.setWordWrap(True)
        layout.addWidget(fallback_intro)
        self.table_point_status_labels: dict[str, QLabel] = {}
        self.table_point_source_labels: dict[str, QLabel] = {}
        for label in TABLE_POINTS:
            box = QFrame()
            box.setObjectName("subPanel")
            box_layout = QVBoxLayout(box)
            title = QLabel(label)
            status = QLabel("Missing")
            source = QLabel("Source: none")
            button = QPushButton(f"Save {label} From Current ESP Pose")
            button.setObjectName("primaryButton")
            button.clicked.connect(lambda _checked=False, point_label=label: self.save_touch_requested.emit(point_label))
            box_layout.addWidget(title)
            box_layout.addWidget(status)
            box_layout.addWidget(source)
            box_layout.addWidget(button)
            self.table_point_status_labels[label] = status
            self.table_point_source_labels[label] = source
            layout.addWidget(box)
        self.table_status = QLabel("No touch points saved.")
        self.table_status.setWordWrap(True)
        self.table_source_status = QLabel("Move from the website first, then save. Source details will appear here.")
        self.table_source_status.setWordWrap(True)
        self.table_z_summary = QLabel("Table Z method: placeholder until at least three points are saved.")
        self.table_z_summary.setWordWrap(True)
        layout.addWidget(self.table_status)
        layout.addWidget(self.table_z_summary)
        layout.addWidget(self.table_source_status)
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
        self.pickup_source_status = QLabel("Move from the website first, then save. The saved source will appear here.")
        self.pickup_source_status.setWordWrap(True)
        form.addRow(button)
        form.addRow(self.pickup_source_status)
        return page

    def _validation_step(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        self.validation_label = QLabel("Clicked pixel: none")
        self.validation_label.setWordWrap(True)
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


def _short_label(label: str) -> str:
    for definition in MARKER_DEFINITIONS.values():
        if definition["label"] == label:
            return str(definition["short"])
    return label
