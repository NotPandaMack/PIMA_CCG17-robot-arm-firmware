from __future__ import annotations

from PySide6.QtCore import Qt, Signal
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
    QScrollArea,
    QSizePolicy,
    QProgressBar,
    QSpinBox,
    QStackedWidget,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from ..calibration_markers import MARKER_DEFINITIONS, mapping_warnings
from ..calibration_manager import fit_side_view_table_z, table_relative_z_values
from ..config import DEFAULT_SIDE_CAMERA_URL, DEFAULT_WORKSPACE
from ..realsense_depth import fit_realsense_table_z, fit_realsense_table_z_two_sample
from .camera_page import CameraView


CALIBRATION_STEPS = [
    ("Camera Placement", "Use the overhead 2D webcam for the table view. It must see the whole table, the robot base, and all four ArUco papers. Use the angled D415 only for depth."),
    ("Define Robot Origin", "In the Top 2D Webcam tab, click the center of the robot's rotating base one time. This becomes robot coordinate X=0, Y=0."),
    ("Four-Point Table Mapping", "Keep the ArUco papers on the table in the 2D webcam view, then scan them. The 2D webcam handles X/Y mapping."),
    ("Workspace Bounds", "These safety limits stop the robot from accepting targets outside the reachable box."),
    ("D415 Depth Calibration", "Use the angled D415 depth camera to learn the table plane and safe claw heights. The GUI never moves or lowers the arm."),
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
    learn_realsense_table_requested = Signal()
    save_realsense_anchor_requested = Signal(str)
    start_auto_calibration_requested = Signal()
    sample_claw_color_requested = Signal(int, int)
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
        self.side_board_detection: dict | None = None
        self.side_fit: dict | None = None
        self.realsense_table_plane: dict | None = None
        self.realsense_samples: list[dict] = []
        self.realsense_low_anchor: dict | None = None
        self.realsense_high_anchor: dict | None = None
        self.realsense_fit: dict | None = None
        self.depth_pending_tip: tuple[int, int] | None = None
        self.depth_metadata: dict | None = None
        self._wizard_state: dict[str, bool] = {}
        self.claw_marker_hsv: dict | None = None
        self.claw_marker: dict | None = None
        self._side_table_clicks: list[tuple[int, int]] = []
        self.validation_pixel: tuple[int, int] | None = None

        self.page_layout = QGridLayout(self)
        self.page_layout.setSpacing(14)

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
        self.depth_color_view = CameraView()
        self.depth_color_view.setMinimumSize(860, 520)
        self.depth_color_view.mapped_clicked.connect(lambda mapped: self.handle_depth_click(int(mapped["pixelX"]), int(mapped["pixelY"])))
        self.depth_view = CameraView()
        self.depth_view.setMinimumSize(860, 520)
        self.depth_view.mapped_clicked.connect(lambda mapped: self.handle_depth_click(int(mapped["pixelX"]), int(mapped["pixelY"])))
        self.plane_view = CameraView()
        self.plane_view.setMinimumSize(860, 520)
        self.plane_view.mapped_clicked.connect(lambda mapped: self.handle_depth_click(int(mapped["pixelX"]), int(mapped["pixelY"])))
        self.side_camera_view = CameraView()
        self.side_camera_view.setMinimumSize(860, 520)
        self.side_camera_view.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.side_camera_view.mapped_clicked.connect(lambda mapped: self.handle_side_click(int(mapped["pixelX"]), int(mapped["pixelY"])))

        self.preview_tabs = QTabWidget()
        top_tab = QWidget()
        top_layout = QVBoxLayout(top_tab)
        top_layout.setContentsMargins(0, 0, 0, 0)
        top_layout.addWidget(self.grid_overlay)
        top_layout.addWidget(self.camera_view, 1)
        self.preview_tabs.addTab(top_tab, "Top 2D Webcam")
        self.preview_tabs.addTab(self.depth_color_view, "D415 Color")
        self.preview_tabs.addTab(self.depth_view, "Depth")
        self.preview_tabs.addTab(self.plane_view, "Table Plane")

        left_layout.addWidget(self.title)
        left_layout.addWidget(self.instructions)
        left_layout.addWidget(self.progress)
        left_layout.addWidget(self.preview_tabs, 1)
        self.page_layout.addWidget(left, 0, 0)

        right = QFrame()
        right.setObjectName("panel")
        right_layout = QVBoxLayout(right)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(0)

        # Scrollable container for wizard + advanced panels so content is never clipped
        right_scroll = QScrollArea()
        right_scroll.setWidgetResizable(True)
        right_scroll.setFrameShape(QFrame.NoFrame)
        right_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        right_inner = QWidget()
        right_inner_layout = QVBoxLayout(right_inner)
        right_inner_layout.setContentsMargins(8, 8, 8, 8)
        right_inner_layout.setSpacing(8)

        self.wizard_panel = self._auto_wizard_panel()
        right_inner_layout.addWidget(self.wizard_panel)
        self.stack = QStackedWidget()
        self.stack.addWidget(self._camera_step())
        self.stack.addWidget(self._origin_step())
        self.stack.addWidget(self._mapping_step())
        self.stack.addWidget(self._workspace_step())
        self.stack.addWidget(self._table_z_step())
        self.stack.addWidget(self._pickup_step())
        self.stack.addWidget(self._validation_step())
        self.stack.addWidget(self._finish_step())
        self.advanced_toggle = QCheckBox("Advanced manual calibration")
        right_inner_layout.addWidget(self.advanced_toggle)
        right_inner_layout.addWidget(self.stack)
        right_inner_layout.addStretch(1)

        nav = QHBoxLayout()
        self.back_button = QPushButton("Back")
        self.save_step_button = QPushButton("Save Step")
        self.next_button = QPushButton("Next")
        self.next_button.setObjectName("primaryButton")
        nav.addWidget(self.back_button)
        nav.addWidget(self.save_step_button)
        nav.addWidget(self.next_button)
        self.advanced_nav = QWidget()
        self.advanced_nav.setLayout(nav)
        right_inner_layout.addWidget(self.advanced_nav)

        right_scroll.setWidget(right_inner)
        right_layout.addWidget(right_scroll, 1)
        self.page_layout.addWidget(right, 0, 1)
        self.page_layout.setColumnStretch(0, 4)
        self.page_layout.setColumnStretch(1, 2)

        self.back_button.clicked.connect(lambda: self.set_step(max(0, self.step_index - 1)))
        self.next_button.clicked.connect(lambda: self.set_step(min(len(CALIBRATION_STEPS) - 1, self.step_index + 1)))
        self.save_step_button.clicked.connect(self.save_step_requested.emit)
        self.grid_overlay.toggled.connect(self.grid_overlay_changed.emit)
        self.advanced_toggle.toggled.connect(self.stack.setVisible)
        self.advanced_toggle.toggled.connect(self.advanced_nav.setVisible)
        self.stack.setVisible(False)
        self.advanced_nav.setVisible(False)
        self.set_step(0)

    def set_step(self, index: int) -> None:
        self.step_index = int(index)
        title, instructions = CALIBRATION_STEPS[self.step_index]
        self.title.setText(title)
        self.instructions.setText(instructions)
        self.stack.setCurrentIndex(self.step_index)
        self.progress.setValue(self.step_index + 1)
        self.page_layout.setColumnStretch(0, 4)
        self.page_layout.setColumnStretch(1, 2)
        self.back_button.setEnabled(self.step_index > 0)
        self.next_button.setEnabled(self.step_index < len(CALIBRATION_STEPS) - 1)
        self.step_changed.emit(self.step_index)

    def capture_click(self, x: int, y: int, debug: dict | None = None) -> None:
        if self.step_index == 1:
            self.origin_pixel = (x, y)
            self.origin_label.setText(f"Origin pixel: {x}, {y}")
            self._emit_marker_overlay_changed()
            self.update_wizard_status()
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
    def handle_depth_click(self, x: int, y: int) -> None:
        self.depth_pending_tip = (x, y)
        self.depth_tip_status.setText(f"Claw tip selected at D415 pixel {x}, {y}. Now press the matching Capture High/Medium/Low Depth Sample button.")
        for view in (self.depth_color_view, self.depth_view, self.plane_view):
            view.set_marker(x, y, "D415 claw tip")
        self.update_wizard_status()

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
        self.update_wizard_status()

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
        if self.side_click_mode.currentData() == "sample_color":
            self.sample_claw_color_requested.emit(x, y)
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
        self.side_board_detection = result
        self.side_board_markers = result.get("markers", []) if isinstance(result.get("markers"), list) else []
        warnings = result.get("warnings", []) if isinstance(result.get("warnings"), list) else []
        warning_text = f" Warnings: {' '.join(warnings)}" if warnings else ""
        board_type = result.get("type", "board")
        corners = result.get("charucoCornerCount")
        quality = result.get("quality")
        if corners is not None:
            self.side_board_status.setText(f"Monitor board detected: {board_type}, {corners} corners, quality {quality}.{warning_text}")
        else:
            self.side_board_status.setText(f"Monitor board markers detected: {len(self.side_board_markers)}.{warning_text}")
        self.update_wizard_status()
        self._update_side_overlay()

    def set_side_camera_status(self, text: str) -> None:
        self.side_camera_status.setText(text)
        self.update_wizard_status()

    def set_side_camera_url(self, stream_url: str) -> None:
        if hasattr(self, "side_stream_url"):
            self.side_stream_url.setText(stream_url)

    def table_z(self) -> dict:
        if self.realsense_fit and self.realsense_fit.get("method") in {"realsense_depth_plane_anchor_fit", "realsense_two_sample"}:
            return self.realsense_fit
        if self.side_fit and self.side_fit.get("method") == "side_view_visual_fit":
            return self.side_fit
        if len(self.table_points) >= 3:
            return {"method": "plane", "points": self.table_points}
        return {"method": "placeholder", "z": 0.0, "points": self.table_points}

    def set_realsense_status(self, text: str) -> None:
        self.realsense_status.setText(text)
        self.update_wizard_status()

    def set_realsense_metadata(self, metadata: dict | None, intrinsics: dict | None, depth_scale: float | None) -> None:
        self.depth_metadata = {"metadata": metadata or {}, "intrinsics": intrinsics or {}, "depthScale": depth_scale}
        serial = (metadata or {}).get("serial", "unknown")
        scale_text = f"{float(depth_scale):.6f}" if isinstance(depth_scale, (int, float)) else "unknown"
        self.depth_info_status.setText(f"D415 metadata: serial {serial}, depth scale {scale_text}.")

    def apply_realsense_table_plane(self, table_plane: dict) -> None:
        self.realsense_table_plane = table_plane
        rms = float(table_plane.get("rmsErrorMm", 0.0))
        inliers = int(table_plane.get("inlierCount", 0))
        total = int(table_plane.get("totalPointCount", inliers))
        coverage = f"{inliers} table pts of {total} total" if total > inliers else f"{inliers} depth points"
        rms_note = "" if rms <= 15.0 else "  ⚠ Too noisy (need < 15 mm) — avoid shiny/dark/transparent surfaces or very bright lights."
        self.depth_table_status.setText(
            f"Table plane learned: {coverage}, RMS {rms:.1f} mm.{rms_note}"
        )
        self._update_realsense_fit()
        self.update_wizard_status()

    def add_realsense_sample(self, status: dict, sample_info: dict, role: str = "anchor") -> None:
        if self.depth_pending_tip is None:
            raise ValueError("click the visible claw tip in the D415 color view before saving")
        if not isinstance(status.get("z"), (int, float)):
            raise ValueError("no valid ESP pose or website IK draft Z is available")
        x, y = self.depth_pending_tip
        sample: dict = {
            "robotZ": float(status["z"]),
            "pixel": {"x": int(x), "y": int(y)},
            "depthMm": float(sample_info["depthMm"]),
            "cameraPointMm": sample_info["cameraPointMm"],
            "heightAboveTableMm": float(sample_info["heightAboveTableMm"]),
            "source": status.get("source", "unknown source"),
        }
        if isinstance(status.get("y"), (int, float)):
            sample["robotY"] = float(status["y"])
        if status.get("manualControlState"):
            sample["manualControlState"] = status["manualControlState"]
        if status.get("espStatus"):
            sample["espStatus"] = status["espStatus"]
        if role == "low_anchor":
            self.realsense_low_anchor = sample
        elif role == "high_anchor":
            self.realsense_high_anchor = sample
        else:
            self.realsense_samples.append(sample)
        self.depth_pending_tip = None
        self.depth_tip_status.setText("Pending D415 claw-tip click: none.")
        self.camera_view.set_marker(None, None)
        self._update_realsense_fit()

    def _update_realsense_fit(self) -> None:
        has_two_sample = self.realsense_low_anchor is not None and self.realsense_high_anchor is not None
        has_multi_sample = self.realsense_table_plane is not None and len(self.realsense_samples) >= 3

        if not self.realsense_table_plane and not has_two_sample:
            self.realsense_fit = None
            low_ok = "✓" if self.realsense_low_anchor else "—"
            high_ok = "✓" if self.realsense_high_anchor else "—"
            self.depth_fit_status.setText(f"Table height: not yet. LOW anchor: {low_ok}  HIGH anchor: {high_ok}")
            self._update_realsense_sample_cards()
            return

        # Two-sample method takes priority (no table plane needed, auto-detects z_axis_inverted)
        if has_two_sample:
            try:
                self.realsense_fit = fit_realsense_table_z_two_sample(
                    low_anchor=self.realsense_low_anchor,
                    high_anchor=self.realsense_high_anchor,
                    safety_margin_mm=self.side_safety_margin.value(),
                )
            except Exception as error:
                self.realsense_fit = None
                self.depth_fit_status.setText(f"Table height: two-sample fit failed. {error}")
                self._update_realsense_sample_cards()
                return
        elif has_multi_sample:
            try:
                self.realsense_fit = fit_realsense_table_z(
                    samples=self.realsense_samples,
                    table_plane=self.realsense_table_plane,
                    safety_margin_mm=self.side_safety_margin.value(),
                    z_axis_inverted=True,
                )
            except Exception as error:
                self.realsense_fit = None
                self.depth_fit_status.setText(f"Table height learned: no. {error}")
                self._update_realsense_sample_cards()
                return
        else:
            self.realsense_fit = None
            low_ok = "✓" if self.realsense_low_anchor else "—"
            high_ok = "✓" if self.realsense_high_anchor else "—"
            self.depth_fit_status.setText(
                f"Table height: not yet. LOW anchor: {low_ok}  HIGH anchor: {high_ok}  "
                f"(or add {max(0, 3 - len(self.realsense_samples))} more multi-samples)"
            )
            self._update_realsense_sample_cards()
            return

        z_axis_inverted = bool(self.realsense_fit.get("zAxisInverted", True))
        z_values = table_relative_z_values(float(self.realsense_fit["z"]), z_axis_inverted=z_axis_inverted)
        self.realsense_fit.update(z_values)
        self.realsense_fit.update(
            {
                "minimumClearanceMm": 10.0,
                "hoverClearanceMm": 60.0,
                "approachClearanceMm": 15.0,
                "liftClearanceMm": 90.0,
            }
        )
        error_mm = float(self.realsense_fit.get("fit", {}).get("errorMm", 0.0))
        method_label = "two-sample" if self.realsense_fit.get("method") == "realsense_two_sample" else "multi-sample"
        warning = f"  ⚠ error {error_mm:.1f} mm — recapture anchors" if error_mm > 5.0 else ""
        slope = self.realsense_fit.get("hoverSlopeZperY")
        slope_text = f", Y-slope {slope:.3f} Z/mm" if isinstance(slope, (int, float)) else " (no Y-slope — anchors at same Y)"
        self.depth_fit_status.setText(
            f"Table height learned ({method_label}): tableZ {self.realsense_fit['z']:.1f} mm, "
            f"hoverZ {self.realsense_fit.get('hoverRefZ', self.realsense_fit['safeHoverZ']):.1f} mm"
            f"{slope_text}, error {error_mm:.1f} mm.{warning}"
        )
        self.validation_summary.setText(
            f"D415 table height ({method_label}). tableZ {self.realsense_fit['z']:.1f}, "
            f"safeHoverZ {self.realsense_fit['safeHoverZ']:.1f}, "
            f"lowApproachZ {self.realsense_fit['lowApproachZ']:.1f}, liftZ {self.realsense_fit['liftZ']:.1f}, "
            f"zAxisInverted {z_axis_inverted}{slope_text}."
        )
        self._update_realsense_sample_cards()
        self.update_wizard_status()

    def _update_realsense_sample_cards(self) -> None:
        lines = []
        if self.realsense_low_anchor:
            s = self.realsense_low_anchor
            y_text = f", Y {s['robotY']:.1f}" if isinstance(s.get("robotY"), (int, float)) else ""
            lines.append(f"LOW:  robotZ {s['robotZ']:.1f}{y_text}, height {s['heightAboveTableMm']:.1f} mm")
        else:
            lines.append("LOW:  — (not captured)")
        if self.realsense_high_anchor:
            s = self.realsense_high_anchor
            y_text = f", Y {s['robotY']:.1f}" if isinstance(s.get("robotY"), (int, float)) else ""
            lines.append(f"HIGH: robotZ {s['robotZ']:.1f}{y_text}, height {s['heightAboveTableMm']:.1f} mm")
        else:
            lines.append("HIGH: — (not captured)")
        for index, sample in enumerate(self.realsense_samples, start=1):
            lines.append(f"S{index}: robotZ {sample['robotZ']:.1f}, height {sample['heightAboveTableMm']:.1f} mm")
        self.depth_samples_status.setText("\n".join(lines) if lines else "No D415 depth anchors saved.")

    def _clear_realsense_anchors(self) -> None:
        self.realsense_low_anchor = None
        self.realsense_high_anchor = None
        self.realsense_samples = []
        self.realsense_fit = None
        self._update_realsense_fit()
        self.update_wizard_status()

    def _update_side_fit(self) -> None:
        if not self.side_table_line or len(self.side_samples) < 3:
            self.side_fit = None
            self.side_fit_status.setText(f"Fit ready: no. Samples: {len(self.side_samples)} / 3 minimum.")
            return
        try:
            self.side_fit = fit_side_view_table_z(
                samples=self.side_samples,
                table_line=self.side_table_line,
                safety_margin_mm=self.side_safety_margin.value(),
                z_axis_inverted=True,
            )
        except Exception as error:
            self.side_fit = None
            self.side_fit_status.setText(f"Fit ready: no. {error}")
            return
        fit = self.side_fit.get("fit", {})
        z_values = table_relative_z_values(float(self.side_fit["z"]), z_axis_inverted=True)
        self.side_fit.update(z_values)
        self.side_fit.update(
            {
                "minimumClearanceMm": 10.0,
                "hoverClearanceMm": 60.0,
                "approachClearanceMm": 15.0,
                "liftClearanceMm": 90.0,
            }
        )
        self.side_fit_status.setText(
            f"Fit ready: yes. visual tableZ {self.side_fit['z']:.1f} mm, "
            f"safety margin {self.side_fit['safetyMarginMm']:.1f} mm, "
            f"{fit.get('pixelsPerRobotMm', 0.0):.2f} px/mm, error {fit.get('errorPx', 0.0):.1f} px."
        )
        self.validation_summary.setText(
            f"Table height learned. tableZ {self.side_fit['z']:.1f}, "
            f"safeHoverZ {self.side_fit['safeHoverZ']:.1f}, "
            f"lowApproachZ {self.side_fit['lowApproachZ']:.1f}, liftZ {self.side_fit['liftZ']:.1f}."
        )
        self.update_wizard_status()

    def _update_side_sample_cards(self) -> None:
        high = min((sample["robotZ"] for sample in self.side_samples), default=None)
        low = max((sample["robotZ"] for sample in self.side_samples), default=None)
        self.side_high_status.setText("High sample saved" if high is not None else "High sample missing")
        self.side_low_status.setText("Low sample saved" if low is not None and high is not None and low > high else "Low sample missing")
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
                board=self.side_board_detection,
                fit=self.side_fit,
            )
            if self.claw_marker:
                pixel = self.claw_marker.get("pixel", {})
                self.side_camera_view.set_marker(int(pixel.get("x", 0)), int(pixel.get("y", 0)), f"claw {float(self.claw_marker.get('confidence', 0.0)):.2f}")
            elif not self.side_pending_tip:
                self.side_camera_view.set_marker(None, None)

    def set_claw_marker_color(self, hsv: dict) -> None:
        self.claw_marker_hsv = hsv
        self.claw_color_status.setText(f"Claw marker color sampled: H {hsv['h']:.0f}, S {hsv['s']:.0f}, V {hsv['v']:.0f}")
        self.update_wizard_status()

    def reset_claw_marker_color(self) -> None:
        self.claw_marker_hsv = None
        self.claw_marker = None
        self.claw_color_status.setText("Claw marker color: not sampled.")
        self._update_side_overlay()
        self.update_wizard_status()

    def set_tracked_claw_marker(self, marker: dict | None) -> None:
        self.claw_marker = marker
        if marker:
            pixel = marker.get("pixel", {})
            self.side_pending_tip = (int(pixel.get("x", 0)), int(pixel.get("y", 0)))
            self.side_tip_status.setText(f"Claw marker tracked: {pixel.get('x')}, {pixel.get('y')} confidence {float(marker.get('confidence', 0.0)):.2f}")
        elif self.auto_track_claw.isChecked():
            self.side_tip_status.setText("Claw marker not tracked. Click the claw tip manually if needed.")
        self._update_side_overlay()
        self.update_wizard_status()

    def auto_track_claw_enabled(self) -> bool:
        return hasattr(self, "auto_track_claw") and self.auto_track_claw.isChecked() and self.claw_marker_hsv is not None

    def update_wizard_status(self, state: dict | None = None) -> None:
        if not hasattr(self, "wizard_status_labels"):
            return
        if state:
            self._wizard_state.update({key: bool(value) for key, value in state.items()})
        state = self._wizard_state
        values = {
            "top_camera": bool(state.get("top_camera")),
            "depth": bool(self.depth_metadata) or bool(state.get("depth")),
            "top_markers": len(self.mapping_pixels) == 4,
            "origin": self.origin_pixel is not None,
            "plane": self.realsense_table_plane is not None,
            "anchors": self.realsense_fit is not None,
            "website": bool(state.get("website")),
            "esp": bool(state.get("esp")),
            "saved": bool(state.get("saved")),
        }
        labels = {
            "top_camera": "2D webcam ready",
            "depth": "D415 depth ready",
            "top_markers": "ArUco X/Y markers detected",
            "origin": "Robot base clicked",
            "plane": "Table plane learned",
            "anchors": "Depth anchors saved",
            "website": "Website connected",
            "esp": "ESP pose readable",
            "saved": "Calibration saved",
        }
        for key, label in labels.items():
            self.wizard_status_labels[key].setText(f"{'[x]' if values[key] else '[ ]'} {label}")
        self.next_action.setText(self._wizard_next_action(values))

    def _wizard_next_action(self, values: dict[str, bool]) -> str:
        for key, text in [
            ("top_camera", "Start Auto Calibration to start the overhead 2D webcam."),
            ("depth", "Connect the angled D415 over USB 3 and wait for depth frames."),
            ("top_markers", "Put the four ArUco papers flat on the table in the 2D webcam view, then scan top markers."),
            ("origin", "In the Top 2D Webcam tab, click the exact center of the robot's rotating base."),
            ("plane", "In the D415 views, make sure the desk surface fills most of the frame, then click Learn Table Plane."),
            ("anchors", "Learn the table plane first (claw clear of frame), then move the claw to LOW (max Y, near table) and HIGH (safe hover) positions. Click the claw tip in D415 Color tab and capture each anchor."),
        ]:
            if not values[key]:
                return text
        if self.realsense_fit:
            return "Review the validation summary and send calibration to the backend."
        return "Need a valid D415 depth Z fit from the saved samples."

    def _auto_wizard_panel(self) -> QWidget:
        page = QFrame()
        page.setObjectName("subPanel")
        layout = QVBoxLayout(page)
        title = QLabel("Auto Calibration Wizard")
        title.setObjectName("sectionTitle")
        self.next_action = QLabel("Press Start Auto Calibration. The 2D webcam handles ArUco X/Y. The angled D415 handles depth for claw/table height.")
        self.next_action.setWordWrap(True)
        self.start_auto_button = QPushButton("Start Auto Calibration")
        self.start_auto_button.setObjectName("primaryButton")
        self.start_auto_button.clicked.connect(self.start_auto_calibration_requested.emit)
        layout.addWidget(title)
        layout.addWidget(self.next_action)
        intro = QLabel(
            "Camera roles:\n"
            "- Top 2D Webcam tab: use this for ArUco papers, robot origin, and X/Y object position.\n"
            "- D415 Color/Depth/Table Plane tabs: use these only for depth, table height, and claw-tip samples.\n"
            "The GUI will not move the robot. Move the arm only from the website controls."
        )
        intro.setWordWrap(True)
        layout.addWidget(intro)
        layout.addWidget(self.start_auto_button)

        grid = QGridLayout()
        self.wizard_status_labels: dict[str, QLabel] = {}
        for index, key in enumerate(("top_camera", "depth", "top_markers", "origin", "plane", "anchors", "website", "esp", "saved")):
            label = QLabel("[ ]")
            label.setWordWrap(True)
            self.wizard_status_labels[key] = label
            grid.addWidget(label, index // 2, index % 2)
        layout.addLayout(grid)

        action_row = QHBoxLayout()
        self.detect_top_button = QPushButton("Scan Top Markers")
        self.learn_depth_plane_button = QPushButton("Learn Table Plane")
        self.detect_side_button = QPushButton("Detect Side Board")
        self.detect_top_button.clicked.connect(self.scan_aruco_requested.emit)
        self.learn_depth_plane_button.clicked.connect(self.learn_realsense_table_requested.emit)
        self.detect_side_button.clicked.connect(self.detect_side_board_requested.emit)
        action_row.addWidget(self.detect_top_button)
        action_row.addWidget(self.learn_depth_plane_button)
        layout.addLayout(action_row)

        depth_box = QFrame()
        depth_box.setObjectName("subPanel")
        depth_layout = QVBoxLayout(depth_box)
        self.realsense_status = QLabel("D415 depth camera stopped.")
        self.realsense_status.setWordWrap(True)
        self.depth_info_status = QLabel("D415 metadata: none.")
        self.depth_info_status.setWordWrap(True)
        self.depth_table_status = QLabel("Table plane learned: no.")
        self.depth_tip_status = QLabel("Pending D415 claw-tip click: none.")
        self.depth_tip_status.setWordWrap(True)
        self.depth_fit_status = QLabel("Table height learned: no. Samples: 0 / 3 minimum.")
        self.depth_fit_status.setWordWrap(True)
        d415_instructions = QLabel(
            "D415 depth steps (two-anchor method):\n"
            "1. Aim D415 so it sees the desk and claw. Click Learn Table Plane (claw out of frame).\n"
            "2. Move arm to MAXIMUM Y reach (farthest pick distance), near table (not touching).\n"
            "   Click claw tip in D415 Color tab → Capture LOW Anchor.\n"
            "3. Move arm HIGH (safe hover height) at a different Y.\n"
            "   Click claw tip → Capture HIGH Anchor.\n"
            "Different Y values enable Y-dependent hover Z (compensates for elbow drop at high Y)."
        )
        d415_instructions.setWordWrap(True)
        depth_layout.addWidget(QLabel("RealSense D415 Angled Depth"))
        depth_layout.addWidget(d415_instructions)
        depth_layout.addWidget(self.realsense_status)
        depth_layout.addWidget(self.depth_info_status)
        depth_layout.addWidget(self.depth_table_status)
        depth_layout.addWidget(self.depth_tip_status)
        depth_layout.addWidget(self.depth_fit_status)
        depth_sample_row = QHBoxLayout()
        low_btn = QPushButton("Capture LOW Anchor\n(arm near table)")
        high_btn = QPushButton("Capture HIGH Anchor\n(arm raised high)")
        clear_anchors_btn = QPushButton("Clear Anchors")
        low_btn.clicked.connect(lambda: self.save_realsense_anchor_requested.emit("low_anchor"))
        high_btn.clicked.connect(lambda: self.save_realsense_anchor_requested.emit("high_anchor"))
        clear_anchors_btn.clicked.connect(self._clear_realsense_anchors)
        depth_sample_row.addWidget(low_btn)
        depth_sample_row.addWidget(high_btn)
        depth_sample_row.addWidget(clear_anchors_btn)
        depth_layout.addLayout(depth_sample_row)
        self.depth_samples_status = QLabel("No D415 depth anchors saved.")
        self.depth_samples_status.setWordWrap(True)
        depth_layout.addWidget(self.depth_samples_status)
        layout.addWidget(depth_box)

        self.side_fallback_toggle = QCheckBox("Show deprecated side-camera fallback")
        layout.addWidget(self.side_fallback_toggle)

        stream_box = QFrame()
        stream_box.setObjectName("subPanel")
        stream_layout = QVBoxLayout(stream_box)
        self.side_stream_url = QLineEdit()
        self.side_stream_url.setPlaceholderText(DEFAULT_SIDE_CAMERA_URL)
        side_button_row = QHBoxLayout()
        start_button = QPushButton("Start Side Camera")
        stop_button = QPushButton("Stop Side Camera")
        start_button.clicked.connect(lambda: self.start_side_camera_requested.emit(self.side_stream_url.text().strip()))
        stop_button.clicked.connect(self.stop_side_camera_requested.emit)
        side_button_row.addWidget(start_button)
        side_button_row.addWidget(stop_button)
        self.side_camera_status = QLabel("Side camera stopped.")
        self.side_camera_status.setWordWrap(True)
        stream_layout.addWidget(QLabel("Side View / DroidCam Camera"))
        stream_layout.addWidget(self.side_stream_url)
        stream_layout.addLayout(side_button_row)
        stream_layout.addWidget(self.side_camera_status)
        layout.addWidget(stream_box)

        controls = QFrame()
        controls.setObjectName("subPanel")
        controls_layout = QFormLayout(controls)
        self.side_click_mode = QComboBox()
        self.side_click_mode.addItem("Click table line points", "table")
        self.side_click_mode.addItem("Click claw tip sample", "tip")
        self.side_click_mode.addItem("Sample claw marker color", "sample_color")
        self.side_safety_margin = self._double(10.0, 0.0, 50.0)
        controls_layout.addRow("Side-view click mode", self.side_click_mode)
        controls_layout.addRow("Safety margin mm", self.side_safety_margin)
        layout.addWidget(controls)

        status_box = QFrame()
        status_box.setObjectName("subPanel")
        status_layout = QVBoxLayout(status_box)
        self.side_board_status = QLabel("Monitor board detected: no.")
        self.side_table_status = QLabel("Table line set: no.")
        self.side_tip_status = QLabel("Pending claw-tip click: none.")
        self.side_high_status = QLabel("High sample missing")
        self.side_low_status = QLabel("Low sample missing")
        self.side_fit_status = QLabel("Fit ready: no. Samples: 0 / 3 minimum.")
        for label in (
            self.side_board_status,
            self.side_table_status,
            self.side_tip_status,
            self.side_high_status,
            self.side_low_status,
            self.side_fit_status,
        ):
            label.setWordWrap(True)
            status_layout.addWidget(label)
        self.side_samples_status = QLabel("No side-view samples saved.")
        self.side_samples_status.setWordWrap(True)
        status_layout.addWidget(self.side_samples_status)
        layout.addWidget(status_box)

        side_claw_widget = QWidget()
        claw_row = QHBoxLayout(side_claw_widget)
        claw_row.setContentsMargins(0, 0, 0, 0)
        self.auto_track_claw = QCheckBox("Auto-track claw marker")
        self.sample_claw_button = QPushButton("Sample claw marker color from click")
        self.reset_claw_button = QPushButton("Reset marker color")
        self.sample_claw_button.clicked.connect(lambda: self.side_click_mode.setCurrentIndex(self.side_click_mode.findData("sample_color")))
        self.reset_claw_button.clicked.connect(self.reset_claw_marker_color)
        claw_row.addWidget(self.auto_track_claw)
        claw_row.addWidget(self.sample_claw_button)
        claw_row.addWidget(self.reset_claw_button)
        layout.addWidget(side_claw_widget)
        self.claw_color_status = QLabel("Claw marker color: not sampled.")
        layout.addWidget(self.claw_color_status)

        side_sample_widget = QWidget()
        sample_row = QHBoxLayout(side_sample_widget)
        sample_row.setContentsMargins(0, 0, 0, 0)
        for text in ("Capture High Sample", "Capture Medium Sample", "Capture Low Sample"):
            button = QPushButton(text)
            button.clicked.connect(self.save_side_sample_requested.emit)
            sample_row.addWidget(button)
        layout.addWidget(side_sample_widget)
        for widget in (stream_box, controls, status_box, side_claw_widget, self.claw_color_status, side_sample_widget):
            widget.setVisible(False)
        self.side_fallback_toggle.toggled.connect(stream_box.setVisible)
        self.side_fallback_toggle.toggled.connect(controls.setVisible)
        self.side_fallback_toggle.toggled.connect(status_box.setVisible)
        self.side_fallback_toggle.toggled.connect(side_claw_widget.setVisible)
        self.side_fallback_toggle.toggled.connect(self.claw_color_status.setVisible)
        self.side_fallback_toggle.toggled.connect(side_sample_widget.setVisible)
        self.validation_summary = QLabel("Validation summary will appear after the Z fit is ready.")
        self.validation_summary.setWordWrap(True)
        layout.addWidget(self.validation_summary)
        self.send_calibration_button = QPushButton("Send Calibration To Website/Backend")
        self.send_calibration_button.setObjectName("primaryButton")
        self.send_calibration_button.clicked.connect(self.finish_requested.emit)
        layout.addWidget(self.send_calibration_button)
        self.update_wizard_status()
        return page

    def set_validation_result(self, text: str) -> None:
        self.validation_result.setText(text)

    def _camera_step(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        self.camera_status_label = QLabel("Start the camera and confirm the table is visible.")
        self.camera_status_label.setText(
            "Physical setup:\n"
            "1. Overhead 2D webcam: directly above the robot arm, looking straight down at the desk.\n"
            "2. ArUco papers: flat on the table in the webcam view.\n"
            "3. D415: high up and tilted down at an angle so it sees the desk surface and the claw.\n"
            "4. Keep both cameras still after calibration starts."
        )
        self.camera_status_label.setWordWrap(True)
        layout.addWidget(self.camera_status_label)
        layout.addStretch(1)
        return page

    def _origin_step(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        self.origin_label = QLabel("Origin pixel: not selected")
        origin_help = QLabel(
            "Use the Top 2D Webcam tab. Click the center of the robot's rotating base once. "
            "Do not click the claw, wrist, or object. This point anchors robot X=0, Y=0 for the webcam mapping."
        )
        origin_help.setWordWrap(True)
        layout.addWidget(origin_help)
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
        self.mapping_help.setText(
            "Use the Top 2D Webcam tab for this step.\n"
            "1. Put the four ArUco papers flat on the table.\n"
            "2. Make sure the overhead webcam can see all four papers at once.\n"
            "3. Click Scan Top Markers. The app fills the marker pixels and computes X/Y mapping.\n"
            "4. Only use manual click fallback if a paper cannot be detected."
        )
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
        intro = QLabel(
            "Workspace bounds are robot safety limits in millimeters. Keep these conservative. "
            "The robot will not accept autonomous pickup targets outside this box."
        )
        intro.setWordWrap(True)
        form.addRow(intro)
        self.workspace_inputs = {}
        for key, value in DEFAULT_WORKSPACE.items():
            widget = self._double(float(value), -1000.0, 1000.0)
            self.workspace_inputs[key] = widget
            form.addRow(key, widget)
        return page

    def _table_z_step(self) -> QWidget:
        page = QWidget()
        layout = QGridLayout(page)
        layout.setColumnStretch(0, 3)
        layout.setColumnStretch(1, 2)
        layout.setRowStretch(1, 1)
        intro = QLabel("Side camera preview, table-line clicks, claw-tip clicks, and visual Z samples are now in the main Auto Calibration Wizard. This advanced section only keeps the old manual touch fallback.")
        intro.setWordWrap(True)
        layout.addWidget(intro, 0, 0, 1, 2)

        fallback_label = QLabel("Advanced manual touch fallback")
        fallback_label.setObjectName("sectionTitle")
        layout.addWidget(fallback_label, 1, 0, 1, 2)
        fallback_intro = QLabel("Only use this if visual side-view calibration is unavailable. Move the arm from the website and save current poses; this GUI still never sends lowering commands.")
        fallback_intro.setWordWrap(True)
        layout.addWidget(fallback_intro, 2, 0, 1, 2)
        self.table_point_status_labels: dict[str, QLabel] = {}
        self.table_point_source_labels: dict[str, QLabel] = {}
        fallback_grid = QGridLayout()
        for index, label in enumerate(TABLE_POINTS):
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
            fallback_grid.addWidget(box, index // 3, index % 3)
        layout.addLayout(fallback_grid, 3, 0, 1, 2)
        self.table_status = QLabel("No touch points saved.")
        self.table_status.setWordWrap(True)
        self.table_source_status = QLabel("Move from the website first, then save. Source details will appear here.")
        self.table_source_status.setWordWrap(True)
        self.table_z_summary = QLabel("Table Z method: placeholder until at least three points are saved.")
        self.table_z_summary.setWordWrap(True)
        layout.addWidget(self.table_status, 4, 0, 1, 2)
        layout.addWidget(self.table_z_summary, 5, 0, 1, 2)
        layout.addWidget(self.table_source_status, 6, 0, 1, 2)
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
