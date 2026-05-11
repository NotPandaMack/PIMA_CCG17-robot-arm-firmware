from __future__ import annotations

import time
from typing import Any

import cv2
from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (
    QFrame,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QStackedWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from .calibration_manager import (
    ChecklistState,
    build_calibration,
    calibration_complete,
    convert_pixel,
    has_homography,
    load_local_calibration,
    pickup_pose_calibrated,
    save_local_calibration,
    table_z_calibrated,
    workspace_bounds_saved,
)
from .camera_worker import CameraWorker
from .config import DEFAULT_HSV_PROFILE, load_settings, normalize_http_url, save_settings
from .detection_worker import DetectionResult, HSVProfile
from .esp_client import EspClient
from .pi_client import PiClient
from .widgets.autonomous_page import AutonomousPage
from .widgets.calibration_page import CalibrationPage
from .widgets.camera_page import CameraPage
from .widgets.connection_page import ConnectionPage
from .widgets.logs_panel import LogsPanel
from .widgets.status_widgets import ChecklistPanel, SafetyBar, StatusPill
from .widgets.test_page import TestPage


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Robot Arm Control Center")
        self.settings = load_settings()
        self.calibration = load_local_calibration()
        self.pi_client = PiClient(self.settings["piUrl"], mock=bool(self.settings.get("mockPi")))
        self.esp_client = EspClient(self.settings["espUrl"], fake=bool(self.settings.get("fakeEsp")))

        self.pi_connected = False
        self.esp_connected = False
        self.webcam_connected = False
        self.last_esp_status: dict[str, Any] | None = None
        self.last_detection: DetectionResult | None = None
        self.last_robot_xy: tuple[float, float] | None = None
        self.last_plan: dict[str, Any] | None = None
        self.last_frame_size: tuple[int, int] | None = None
        self.last_target_status = "none"
        self.last_send_at = 0.0
        self.camera_worker: CameraWorker | None = None

        self._build_ui()
        self._connect_signals()
        self._apply_settings_to_ui()
        self._apply_style()
        self._start_status_timer()
        self.refresh_status()

        start_page = "health" if self._is_configured() else "setup"
        self.go_to(start_page)

    def closeEvent(self, event: Any) -> None:
        self.stop_camera()
        self.settings["lastPage"] = self.current_page_name()
        save_settings(self.settings)
        super().closeEvent(event)

    def _build_ui(self) -> None:
        central = QWidget()
        root = QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)
        self.setCentralWidget(central)

        self.safety_bar = SafetyBar()
        root.addWidget(self.safety_bar)

        body = QHBoxLayout()
        body.setContentsMargins(12, 12, 12, 8)
        body.setSpacing(12)
        root.addLayout(body, 1)

        nav = QFrame()
        nav.setObjectName("sidebar")
        nav_layout = QVBoxLayout(nav)
        nav_layout.setContentsMargins(10, 10, 10, 10)
        nav_layout.setSpacing(8)
        self.nav_buttons: dict[str, QPushButton] = {}
        for key, label in [
            ("setup", "Setup"),
            ("health", "Health"),
            ("camera", "Camera"),
            ("calibration", "Calibration"),
            ("test", "Manual Test"),
            ("preview", "Pickup Preview"),
            ("autonomous", "Autonomous"),
        ]:
            button = QPushButton(label)
            button.setCheckable(True)
            button.clicked.connect(lambda _checked=False, page=key: self.go_to(page))
            self.nav_buttons[key] = button
            nav_layout.addWidget(button)
        nav_layout.addStretch(1)
        body.addWidget(nav)

        self.stack = QStackedWidget()
        self.setup_checklist = ChecklistPanel()
        self.health_checklist = ChecklistPanel()
        self.connection_page = ConnectionPage(self.setup_checklist)
        self.health_page = self._make_health_page()
        self.camera_page = CameraPage()
        self.calibration_page = CalibrationPage()
        self.test_page = TestPage()
        self.preview_page = self._make_preview_page()
        self.autonomous_page = AutonomousPage()
        self.pages = {
            "setup": self.connection_page,
            "health": self.health_page,
            "camera": self.camera_page,
            "calibration": self.calibration_page,
            "test": self.test_page,
            "preview": self.preview_page,
            "autonomous": self.autonomous_page,
        }
        for page in self.pages.values():
            self.stack.addWidget(page)
        body.addWidget(self.stack, 1)

        self.logs = LogsPanel()
        self.logs.setMaximumHeight(170)
        root.addWidget(self.logs)

    def _make_health_page(self) -> QWidget:
        page = QWidget()
        layout = QHBoxLayout(page)
        layout.setSpacing(14)

        cards = QFrame()
        cards.setObjectName("panel")
        cards_layout = QVBoxLayout(cards)
        title = QLabel("System Health")
        title.setObjectName("sectionTitle")
        cards_layout.addWidget(title)
        self.health_pills = {
            "pi": StatusPill("Pi offline", "red"),
            "esp": StatusPill("ESP offline", "red"),
            "estop": StatusPill("ESTOP unknown", "yellow"),
            "camera": StatusPill("Webcam offline", "red"),
            "calibration": StatusPill("Not calibrated", "red"),
            "motion": StatusPill("Motion disabled", "yellow"),
            "target": StatusPill("Target none", "yellow"),
        }
        for pill in self.health_pills.values():
            cards_layout.addWidget(pill)
        refresh = QPushButton("Refresh Status")
        refresh.setObjectName("primaryButton")
        refresh.clicked.connect(self.refresh_status)
        cards_layout.addWidget(refresh)
        cards_layout.addStretch(1)
        layout.addWidget(cards, 1)
        layout.addWidget(self.health_checklist, 1)
        return page

    def _make_preview_page(self) -> QWidget:
        page = QWidget()
        layout = QHBoxLayout(page)
        left = QFrame()
        left.setObjectName("panel")
        left_layout = QVBoxLayout(left)
        title = QLabel("Pickup Preview")
        title.setObjectName("sectionTitle")
        self.preview_target_label = QLabel("No detected target.")
        self.preview_target_label.setWordWrap(True)
        self.preview_button = QPushButton("Generate Preview")
        self.hover_preview_button = QPushButton("Generate Hover-Only Preview")
        self.preview_button.setObjectName("primaryButton")
        self.preview_button.clicked.connect(lambda: self.generate_preview(False))
        self.hover_preview_button.clicked.connect(lambda: self.generate_preview(True))
        left_layout.addWidget(title)
        left_layout.addWidget(self.preview_target_label)
        left_layout.addWidget(self.preview_button)
        left_layout.addWidget(self.hover_preview_button)
        left_layout.addStretch(1)
        layout.addWidget(left, 1)

        right = QFrame()
        right.setObjectName("panel")
        right_layout = QVBoxLayout(right)
        right_layout.addWidget(QLabel("Generated ESP command timeline"))
        self.preview_text = QTextEdit()
        self.preview_text.setReadOnly(True)
        right_layout.addWidget(self.preview_text, 1)
        layout.addWidget(right, 2)
        return page

    def _connect_signals(self) -> None:
        self.connection_page.save_requested.connect(self.save_setup_settings)
        self.connection_page.test_pi_requested.connect(self.test_pi_connection)
        self.connection_page.test_esp_requested.connect(self.test_esp_connection)
        self.connection_page.test_camera_requested.connect(self.test_webcam)
        self.connection_page.auto_detect_requested.connect(self.auto_detect_pi)
        self.connection_page.start_setup_requested.connect(lambda: self.go_to("setup"))

        self.camera_page.start_camera_requested.connect(self.start_camera)
        self.camera_page.stop_camera_requested.connect(self.stop_camera)
        self.camera_page.send_target_requested.connect(self.send_detected_target)
        self.camera_page.save_hsv_requested.connect(self.save_hsv_profile)
        self.camera_page.reset_hsv_requested.connect(self.reset_hsv_profile)
        self.camera_page.profile_changed.connect(self.update_hsv_profile)
        self.camera_page.continuous_changed.connect(self.set_continuous_send)
        self.camera_page.rate_changed.connect(self.set_send_rate)

        self.calibration_page.camera_view.clicked.connect(self.handle_calibration_click)
        self.calibration_page.save_touch_requested.connect(self.save_table_touch_point)
        self.calibration_page.save_pickup_requested.connect(self.save_pickup_pose_from_esp)
        self.calibration_page.save_step_requested.connect(self.save_calibration_step)
        self.calibration_page.preview_hover_requested.connect(lambda: self.generate_preview(True))
        self.calibration_page.test_hover_requested.connect(lambda: self.run_pick(True))
        self.calibration_page.finish_requested.connect(self.finish_calibration)

        self.test_page.command_requested.connect(self.send_esp_command)
        self.test_page.refresh_requested.connect(self.refresh_status)
        self.test_page.enable_motion_requested.connect(self.enable_motion)

        self.autonomous_page.preview_requested.connect(self.generate_preview)
        self.autonomous_page.pick_requested.connect(self.run_pick)
        self.autonomous_page.auto_pick_changed.connect(self.set_auto_pick_enabled)

    def _apply_settings_to_ui(self) -> None:
        self.connection_page.set_from_settings(self.settings)
        profile = HSVProfile.from_settings(self.settings.get("hsv", DEFAULT_HSV_PROFILE))
        self.camera_page.set_profile(profile)
        self.camera_page.continuous.setChecked(bool(self.settings.get("continuousSend", False)))
        self.camera_page.rate.setValue(int(float(self.settings.get("sendRateHz", 5.0))))

    def _start_status_timer(self) -> None:
        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self.refresh_status)
        self.status_timer.start(2500)

        self.auto_pick_timer = QTimer(self)
        self.auto_pick_timer.timeout.connect(self._auto_pick_tick)
        self.auto_pick_timer.start(500)
        self._stable_since: float | None = None
        self._last_auto_pick_at = 0.0

    def go_to(self, page: str) -> None:
        self.stack.setCurrentWidget(self.pages[page])
        for key, button in self.nav_buttons.items():
            button.setChecked(key == page)
        self.settings["lastPage"] = page

    def current_page_name(self) -> str:
        current = self.stack.currentWidget()
        for key, page in self.pages.items():
            if page is current:
                return key
        return "setup"

    def save_setup_settings(self) -> None:
        patch = self.connection_page.to_settings_patch()
        patch["piUrl"] = normalize_http_url(patch["piUrl"], with_port=True)
        patch["espUrl"] = normalize_http_url(patch["espUrl"], with_port=False)
        self.settings.update(patch)
        self.pi_client.set_base_url(self.settings["piUrl"])
        self.pi_client.set_mock(bool(self.settings.get("mockPi")))
        self.esp_client.set_base_url(self.settings["espUrl"])
        self.esp_client.set_fake(bool(self.settings.get("fakeEsp")))
        save_settings(self.settings)
        self.log("Settings saved.")
        self.connection_page.set_from_settings(self.settings)
        self.refresh_status()

    def test_pi_connection(self) -> None:
        self.save_setup_settings()
        try:
            self.pi_client.health()
            self.pi_connected = True
            self.log("Pi server connected.")
            self._load_pi_config_and_calibration()
        except Exception as error:
            self.pi_connected = False
            self.log(f"Pi server unreachable: {error}")
        self.update_ui_state()

    def test_esp_connection(self) -> None:
        self.save_setup_settings()
        try:
            self.last_esp_status = self.esp_client.status()
            self.esp_connected = True
            self.log("ESP connected.")
        except Exception as error:
            self.esp_connected = False
            self.last_esp_status = None
            self.log(f"ESP unreachable: {error}")
        self.update_ui_state()

    def test_webcam(self) -> None:
        index = int(self.settings.get("cameraIndex", 0))
        cap = cv2.VideoCapture(index)
        ok = cap.isOpened()
        if ok:
            frame_ok, frame = cap.read()
            ok = bool(frame_ok)
            if frame_ok:
                self.last_frame_size = (frame.shape[1], frame.shape[0])
        cap.release()
        self.webcam_connected = ok
        self.log(f"Webcam {index} {'connected' if ok else 'failed to open'}.")
        self.update_ui_state()

    def auto_detect_pi(self) -> None:
        candidates = [self.settings.get("piUrl", ""), "http://raspberrypi.local:8000"]
        for candidate in candidates:
            self.pi_client.set_base_url(candidate)
            try:
                self.pi_client.health()
                self.settings["piUrl"] = candidate
                self.connection_page.set_from_settings(self.settings)
                self.pi_connected = True
                self.log(f"Auto-detected Pi server at {candidate}.")
                self.update_ui_state()
                return
            except Exception:
                continue
        self.log("Auto-detect did not find a Pi server. Enter the Pi URL manually.")
        self.update_ui_state()

    def refresh_status(self) -> None:
        try:
            self.pi_client.set_mock(bool(self.settings.get("mockPi")))
            self.pi_client.health()
            self.pi_connected = True
            self._load_pi_config_and_calibration()
        except Exception:
            self.pi_connected = False

        try:
            self.esp_client.set_fake(bool(self.settings.get("fakeEsp")))
            self.last_esp_status = self.esp_client.status()
            self.esp_connected = True
        except Exception:
            self.esp_connected = False
            self.last_esp_status = None

        self.update_ui_state()

    def _load_pi_config_and_calibration(self) -> None:
        try:
            config = self.pi_client.get_config().get("config", {})
            self.settings["motionEnabled"] = bool(config.get("motionEnabled", self.settings.get("motionEnabled", False)))
            self.settings["workspace"] = config.get("workspace", self.settings.get("workspace", {}))
            if config.get("espBaseUrl") and "ESP8266_IP" not in str(config.get("espBaseUrl")):
                self.settings["espUrl"] = config["espBaseUrl"]
                self.esp_client.set_base_url(config["espBaseUrl"])
        except Exception as error:
            self.log(f"Pi config check failed: {error}")
        try:
            calibration = self.pi_client.get_calibration()
            if isinstance(calibration, dict) and calibration.get("homography"):
                self.calibration.update(calibration)
        except Exception as error:
            self.log(f"Pi calibration check failed: {error}")

    def start_camera(self) -> None:
        if self.camera_worker and self.camera_worker.isRunning():
            return
        profile = self.camera_page.profile()
        self.camera_worker = CameraWorker(int(self.settings.get("cameraIndex", 0)))
        self.camera_worker.configure(profile=profile, homography=self.calibration.get("homography") if has_homography(self.calibration) else None)
        self.camera_worker.frame_ready.connect(self.on_camera_frame)
        self.camera_worker.detection_ready.connect(self.on_detection)
        self.camera_worker.camera_status.connect(self.on_camera_status)
        self.camera_worker.start()
        self.log("Camera starting.")

    def stop_camera(self) -> None:
        if self.camera_worker:
            self.camera_worker.stop()
            self.camera_worker = None
        self.webcam_connected = False
        self.log("Camera stopped.")
        self.update_ui_state()

    def on_camera_status(self, online: bool, message: str) -> None:
        self.webcam_connected = online
        self.log(message)
        self.update_ui_state()

    def on_camera_frame(self, frame: Any) -> None:
        h, w = frame.shape[:2]
        self.last_frame_size = (w, h)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image = QImage(rgb.data, w, h, rgb.strides[0], QImage.Format_RGB888).copy()
        pixmap = QPixmap.fromImage(image)
        for view in (self.camera_page.camera_view, self.calibration_page.camera_view, self.autonomous_page.camera_view):
            view.set_frame(pixmap, (w, h))

    def on_detection(self, payload: dict[str, Any]) -> None:
        detection = payload["detection"]
        robot_xy = payload.get("robot_xy")
        self.last_detection = detection
        self.last_robot_xy = robot_xy
        if detection.found:
            text = f"Green object: pixel X {detection.pixel_x}, Y {detection.pixel_y}, confidence {detection.confidence:.2f}, area {detection.area:.0f}"
            if robot_xy:
                robot_text = f"Robot coordinate: X {robot_xy[0]:.1f} mm, Y {robot_xy[1]:.1f} mm. Target valid if inside workspace."
                self.last_target_status = "valid" if detection.confidence >= 0.75 else "low confidence"
            else:
                robot_text = "Object detected in camera pixels only. Calibration required before robot movement."
                self.last_target_status = "pixels"
            self.camera_page.update_detection(text, robot_text, can_send=True)
            self.preview_target_label.setText(f"{text}\n{robot_text}")
            self.autonomous_page.update_target(f"{text}\n{robot_text}")
            if self.settings.get("continuousSend"):
                self._rate_limited_send_target()
        else:
            self.last_target_status = "none"
            self.camera_page.update_detection("No object detected.", "Object detected in camera pixels only. Calibration required before robot movement.", False)
            self.preview_target_label.setText("No detected target.")
            self.autonomous_page.update_target("No detected target.")
        self.update_ui_state()

    def _rate_limited_send_target(self) -> None:
        interval = 1.0 / max(0.1, float(self.settings.get("sendRateHz", 5.0)))
        if time.monotonic() - self.last_send_at >= interval:
            self.send_detected_target(silent=True)

    def send_detected_target(self, silent: bool = False) -> None:
        if not self.last_detection or not self.last_detection.found:
            if not silent:
                self.log("Target not sent: no object detected.")
            return
        payload = self.last_detection.as_payload(self.last_robot_xy)
        if payload is None:
            return
        try:
            result = self.pi_client.post_target(payload)
            self.last_send_at = time.monotonic()
            target = result.get("target", payload)
            self.last_target_status = "valid" if target.get("robotX") is not None else "pixels"
            if not silent:
                self.log("Detected target sent to Pi.")
        except Exception as error:
            self.log(f"Target not sent: Pi rejected or unreachable: {error}")
        self.update_ui_state()

    def update_hsv_profile(self, profile: HSVProfile) -> None:
        self.settings["hsv"] = profile.to_settings()
        if self.camera_worker:
            self.camera_worker.configure(profile=profile)

    def save_hsv_profile(self) -> None:
        self.settings["hsv"] = self.camera_page.profile().to_settings()
        save_settings(self.settings)
        self.log("HSV profile saved.")

    def reset_hsv_profile(self) -> None:
        profile = HSVProfile.from_settings(DEFAULT_HSV_PROFILE)
        self.camera_page.set_profile(profile)
        self.update_hsv_profile(profile)
        self.log("HSV profile reset to green-object defaults.")

    def set_continuous_send(self, enabled: bool) -> None:
        self.settings["continuousSend"] = enabled
        self.log("Continuous target sending enabled." if enabled else "Continuous target sending disabled.")

    def set_send_rate(self, value: float) -> None:
        self.settings["sendRateHz"] = value

    def handle_calibration_click(self, x: int, y: int) -> None:
        self.calibration_page.capture_click(x, y)
        if self.calibration_page.step_index == 6:
            robot_xy = convert_pixel(self.calibration, x, y)
            if robot_xy:
                self.calibration_page.set_validation_result(f"Robot coordinate: X {robot_xy[0]:.1f} mm, Y {robot_xy[1]:.1f} mm")
                payload = {
                    "type": "object_detected",
                    "object": "calibration_validation",
                    "pixelX": x,
                    "pixelY": y,
                    "robotX": robot_xy[0],
                    "robotY": robot_xy[1],
                    "confidence": 1.0,
                }
                try:
                    self.pi_client.post_target(payload)
                    self.log("Calibration validation target sent to Pi.")
                except Exception as error:
                    self.log(f"Validation target could not be sent: {error}")
            else:
                self.calibration_page.set_validation_result("Conversion unavailable. Complete camera mapping first.")

    def save_calibration_step(self) -> None:
        step = self.calibration_page.step_index
        if step == 0:
            if not self.webcam_connected or not self.last_frame_size:
                self.log("Calibration step blocked: camera is not running.")
                return
            self.log("Camera placement step saved.")
        elif step == 1:
            if not self.calibration_page.origin_pixel:
                self.log("Calibration step blocked: click the robot origin first.")
                return
            self.calibration["origin"] = {"pixel": {"x": self.calibration_page.origin_pixel[0], "y": self.calibration_page.origin_pixel[1]}}
            self.log("Robot origin saved.")
        elif step == 2:
            points = self.calibration_page.mapping_points()
            if len(points) != 4:
                self.log("Camera mapping blocked: all four marker pixels are required.")
                return
            self.calibration["points"] = points
            self.log("Four-point mapping saved. Homography will be computed on finish.")
        elif step == 3:
            workspace = self.calibration_page.workspace()
            self.calibration["workspaceBounds"] = workspace
            self.settings["workspace"] = workspace
            try:
                self.pi_client.put_config({"workspace": workspace})
                self.log("Workspace bounds saved to Pi.")
            except Exception as error:
                self.log(f"Workspace saved locally but not uploaded to Pi: {error}")
        elif step == 4:
            self.calibration["tableZ"] = self.calibration_page.table_z()
            self.log("Table Z step saved.")
        elif step == 5:
            self.calibration.update(self.calibration_page.pickup_values())
            self.log("Pickup pose values saved.")
        self.update_ui_state()

    def save_table_touch_point(self, label: str) -> None:
        try:
            status = self.esp_client.status()
            self.last_esp_status = status
            self.esp_connected = True
            self.calibration_page.add_table_point(label, status)
            self.calibration["tableZ"] = self.calibration_page.table_z()
            self.log(f"Saved table touch point: {label}.")
        except Exception as error:
            self.log(f"Touch point not saved: ESP status unavailable: {error}")
        self.update_ui_state()

    def save_pickup_pose_from_esp(self) -> None:
        try:
            status = self.esp_client.status()
            self.last_esp_status = status
            self.esp_connected = True
            if "pitch" not in status or "z" not in status:
                self.log("Pickup pose blocked: ESP status does not include pitch and Z.")
                return
            self.calibration_page.set_pickup_from_status(status)
            self.calibration.update(self.calibration_page.pickup_values())
            self.log("Pickup pose captured from ESP status.")
        except Exception as error:
            self.log(f"Pickup pose not saved: {error}")
        self.update_ui_state()

    def finish_calibration(self) -> None:
        if not self.last_frame_size:
            self.log("Calibration cannot finish: camera frame size is unknown.")
            return
        if self.calibration_page.origin_pixel is None:
            self.log("Calibration cannot finish: robot origin is missing.")
            return
        try:
            calibration = build_calibration(
                camera_width=self.last_frame_size[0],
                camera_height=self.last_frame_size[1],
                origin_pixel=self.calibration_page.origin_pixel,
                points=self.calibration_page.mapping_points(),
                workspace=self.calibration_page.workspace(),
                table_z=self.calibration_page.table_z(),
                pickup=self.calibration_page.pickup_values(),
            )
        except Exception as error:
            self.log(f"Calibration cannot finish: {error}")
            return
        if not calibration_complete(calibration):
            self.log("Calibration incomplete: mapping, workspace, table Z, and pickup pose are all required.")
            return
        self.calibration = calibration
        save_local_calibration(self.calibration)
        config_patch = {
            "workspace": self.calibration["workspaceBounds"],
            "safeHoverZ": float(self.calibration["hoverZ"]),
            "grabOffsetZ": float(self.calibration["grabOffsetZ"]),
            "liftZ": float(self.calibration["liftZ"]),
            "closeClawDegrees": int(self.calibration["clawClosedValue"]),
        }
        try:
            self.pi_client.put_calibration(self.calibration)
            self.pi_client.put_config(config_patch)
            self.log("Calibration saved locally and uploaded to Pi.")
        except Exception as error:
            self.log(f"Calibration saved locally but Pi upload failed: {error}")
        if self.camera_worker:
            self.camera_worker.configure(homography=self.calibration.get("homography"))
        self.update_ui_state()

    def generate_preview(self, hover_only: bool = False) -> None:
        if not self._preview_gate():
            return
        try:
            plan = self.pi_client.preview_pick(hover_only)
            self.last_plan = plan
            self._show_plan(plan)
            if plan.get("ok") and not plan.get("motionEnabled"):
                self.settings["ghostPreviewPassed"] = True
                save_settings(self.settings)
                self.log("Ghost preview generated successfully.")
            elif plan.get("ok"):
                self.log("Preview generated successfully.")
            else:
                self.log("Preview blocked: " + "; ".join(plan.get("errors", [])))
        except Exception as error:
            self.log(f"Preview failed: {error}")
        self.update_ui_state()

    def run_pick(self, hover_only: bool) -> None:
        state = self.checklist_state()
        if hover_only:
            if not (state.motion_enabled and state.ghost_preview_passed and state.target_valid and state.estop_inactive):
                self.log("Hover-only movement blocked: complete calibration, ghost preview, target, motion, and ESTOP checks first.")
                return
            if not self.settings.get("firstHoverConfirmed"):
                if QMessageBox.question(self, "Run first hover movement?", "The robot will move only to safe hover above the target. Continue?") != QMessageBox.StandardButton.Yes:
                    return
                self.settings["firstHoverConfirmed"] = True
        else:
            if not state.full_pickup_ready:
                self.log("Full pickup locked: hover-only real movement must pass first.")
                return
            if not self.settings.get("firstPickupConfirmed"):
                if QMessageBox.question(self, "Run first full pickup?", "The robot will run the full pickup timeline. Continue?") != QMessageBox.StandardButton.Yes:
                    return
                self.settings["firstPickupConfirmed"] = True

        try:
            plan = self.pi_client.pick(hover_only)
            self.last_plan = plan
            self._show_plan(plan)
            if plan.get("sent") and hover_only:
                if QMessageBox.question(self, "Confirm hover-only result", "Did the arm only move to a safe hover above the target?") == QMessageBox.StandardButton.Yes:
                    self.settings["hoverOnlyMovementPassed"] = True
                    self.log("Hover-only movement marked as passed.")
                else:
                    self.settings["hoverOnlyMovementPassed"] = False
                    self.log("Hover-only movement was not marked as passed; full pickup remains locked.")
            elif plan.get("sent"):
                self.log("Full pickup command sent.")
            elif plan.get("ok") and not plan.get("motionEnabled"):
                self.log("Motion disabled: ghost mode only.")
            else:
                self.log("Pickup blocked: " + "; ".join(plan.get("errors", [])))
        except Exception as error:
            self.log(f"Pickup failed: {error}")
        save_settings(self.settings)
        self.update_ui_state()

    def enable_motion(self) -> None:
        state = self.checklist_state()
        if not (state.camera_calibration_complete and state.workspace_bounds_saved and state.table_z_calibrated and state.pickup_pose_calibrated):
            self.log("Motion enable blocked: calibration is incomplete.")
            return
        if not state.ghost_preview_passed:
            self.log("Motion enable blocked: run a successful ghost preview first.")
            return
        if not state.estop_inactive:
            self.log("Motion enable blocked: ESTOP is active.")
            return
        if QMessageBox.question(self, "Enable real motion?", "This allows the Pi server to send movement commands to the ESP. Continue?") != QMessageBox.StandardButton.Yes:
            return
        try:
            response = self.pi_client.put_config({"motionEnabled": True, "espBaseUrl": self.settings["espUrl"]})
            self.settings["motionEnabled"] = bool(response.get("config", {}).get("motionEnabled", True))
            save_settings(self.settings)
            self.log("Motion enabled. Run hover-only movement before full pickup.")
        except Exception as error:
            self.log(f"Motion enable failed: {error}")
        self.update_ui_state()

    def send_esp_command(self, command: str) -> None:
        if command not in {"STOP", "ESTOP", "CLEAR_ESTOP", "CLEAR_TIMELINE"} and not self.esp_connected and not self.settings.get("fakeEsp"):
            self.log(f"Command not sent: ESP is not connected ({command}).")
            return
        try:
            self.esp_client.send_command(command)
            self.log(f"ESP command sent: {command}")
            self.test_esp_connection()
        except Exception as error:
            self.log(f"ESP command failed: {error}")

    def set_auto_pick_enabled(self, enabled: bool) -> None:
        if enabled:
            state = self.checklist_state()
            if not state.full_pickup_ready:
                self.autonomous_page.auto_pick.setChecked(False)
                self.log("Auto-pick blocked: full pickup is not ready.")
                return
            if not self.settings.get("autoPickConfirmed"):
                if QMessageBox.question(self, "Enable auto-pick?", "Auto-pick will trigger after a stable valid object and cooldown. Continue?") != QMessageBox.StandardButton.Yes:
                    self.autonomous_page.auto_pick.setChecked(False)
                    return
                self.settings["autoPickConfirmed"] = True
            self.log("Auto-pick enabled.")
        else:
            self.log("Auto-pick disabled.")
        self.settings["autoPickEnabled"] = enabled
        save_settings(self.settings)

    def _auto_pick_tick(self) -> None:
        if not self.settings.get("autoPickEnabled"):
            self._stable_since = None
            return
        state = self.checklist_state()
        if not state.full_pickup_ready or not self.last_detection or not self.last_detection.found:
            self._stable_since = None
            return
        now = time.monotonic()
        if self._stable_since is None:
            self._stable_since = now
            return
        stable_sec = float(self.settings.get("autoPickStableSec", 2.0))
        cooldown = float(self.autonomous_page.cooldown.value())
        if now - self._stable_since >= stable_sec and now - self._last_auto_pick_at >= cooldown:
            self._last_auto_pick_at = now
            self.log("Auto-pick trigger: object stable and safety checks passed.")
            self.run_pick(False)

    def _preview_gate(self) -> bool:
        if not self.pi_connected and not self.settings.get("mockPi"):
            self.log("Preview blocked: Pi server is not connected.")
            return False
        if not calibration_complete(self.calibration):
            self.log("Preview blocked: calibration is incomplete.")
            return False
        if not self.last_detection or not self.last_detection.found:
            self.log("Preview blocked: no detected object.")
            return False
        if self.last_robot_xy is None:
            self.log("Preview blocked: target has no robot coordinates.")
            return False
        self.send_detected_target(silent=True)
        return True

    def checklist_state(self) -> ChecklistState:
        estop_active = bool(self.last_esp_status.get("estop")) if self.last_esp_status else False
        return ChecklistState(
            pi_connected=self.pi_connected or bool(self.settings.get("mockPi")),
            esp_connected=self.esp_connected or bool(self.settings.get("fakeEsp")),
            webcam_connected=self.webcam_connected,
            object_detection_working=bool(self.last_detection and self.last_detection.found),
            camera_calibration_complete=has_homography(self.calibration),
            workspace_bounds_saved=workspace_bounds_saved(self.calibration),
            table_z_calibrated=table_z_calibrated(self.calibration),
            pickup_pose_calibrated=pickup_pose_calibrated(self.calibration),
            ghost_preview_passed=bool(self.settings.get("ghostPreviewPassed")),
            hover_only_movement_passed=bool(self.settings.get("hoverOnlyMovementPassed")),
            motion_enabled=bool(self.settings.get("motionEnabled")),
            target_valid=self.last_target_status == "valid",
            estop_inactive=not estop_active,
        )

    def update_ui_state(self) -> None:
        state = self.checklist_state()
        calibrated = calibration_complete(self.calibration)
        estop_active = bool(self.last_esp_status.get("estop")) if self.last_esp_status else None
        self.safety_bar.update_state(
            motion_enabled=bool(self.settings.get("motionEnabled")),
            estop_active=estop_active,
            calibrated=calibrated,
            target_status=self.last_target_status,
        )
        self.setup_checklist.update_state(state)
        self.health_checklist.update_state(state)
        self.connection_page.update_statuses(state.pi_connected, state.esp_connected, state.webcam_connected)
        self.test_page.update_status(self.last_esp_status)
        self.test_page.set_motion_button_enabled(
            state.camera_calibration_complete
            and state.workspace_bounds_saved
            and state.table_z_calibrated
            and state.pickup_pose_calibrated
            and state.ghost_preview_passed
            and state.estop_inactive
            and not state.motion_enabled
        )
        self.preview_button.setEnabled(state.target_valid and calibrated)
        self.hover_preview_button.setEnabled(state.target_valid and calibrated)
        self.autonomous_page.set_buttons(
            can_preview=state.target_valid and calibrated,
            can_hover=state.motion_enabled and state.ghost_preview_passed and state.target_valid and state.estop_inactive,
            can_pick=state.full_pickup_ready,
        )
        self.health_pills["pi"].set_state("Pi online" if state.pi_connected else "Pi offline", "green" if state.pi_connected else "red")
        self.health_pills["esp"].set_state("ESP online" if state.esp_connected else "ESP offline", "green" if state.esp_connected else "red")
        self.health_pills["estop"].set_state("ESTOP active" if estop_active else "ESTOP inactive", "red" if estop_active else "green")
        self.health_pills["camera"].set_state("Webcam online" if state.webcam_connected else "Webcam offline", "green" if state.webcam_connected else "red")
        self.health_pills["calibration"].set_state("Calibrated" if calibrated else "Not calibrated", "green" if calibrated else "red")
        self.health_pills["motion"].set_state("Motion enabled" if state.motion_enabled else "Motion disabled", "green" if state.motion_enabled else "yellow")
        self.health_pills["target"].set_state(f"Target {self.last_target_status}", "green" if state.target_valid else "yellow")

    def _show_plan(self, plan: dict[str, Any]) -> None:
        self.autonomous_page.update_plan(plan)
        commands = plan.get("commands", [])
        calculated = plan.get("calculated", {})
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
            "",
            "Commands:",
            *(commands or ["(no commands)"]),
        ]
        errors = plan.get("errors", [])
        if errors:
            lines.insert(10, "Blocked: " + "; ".join(errors))
        self.preview_text.setPlainText("\n".join(lines))

    def _is_configured(self) -> bool:
        return bool(self.settings.get("piUrl") and self.settings.get("espUrl"))

    def log(self, message: str) -> None:
        self.logs.add(message)

    def _apply_style(self) -> None:
        self.setStyleSheet(
            """
            QWidget {
                background: #101419;
                color: #edf2f7;
                font-family: Inter, Segoe UI, Arial, sans-serif;
                font-size: 14px;
            }
            QFrame#safetyBar {
                background: #171d24;
                border-bottom: 1px solid #2f3a46;
            }
            QLabel#appTitle {
                font-size: 20px;
                font-weight: 700;
            }
            QFrame#sidebar, QFrame#panel, QFrame#heroPanel, QFrame#checklistPanel {
                background: #171d24;
                border: 1px solid #2b3440;
                border-radius: 8px;
            }
            QFrame#subPanel {
                background: #121820;
                border: 1px solid #2b3440;
                border-radius: 6px;
            }
            QLabel#heroTitle {
                font-size: 28px;
                font-weight: 800;
            }
            QLabel#sectionTitle {
                font-size: 20px;
                font-weight: 700;
            }
            QLabel#mutedText {
                color: #9aa6b2;
            }
            QLabel#cameraView {
                background: #080a0d;
                border: 1px solid #2b3440;
                border-radius: 8px;
                color: #9aa6b2;
            }
            QPushButton {
                background: #25303b;
                border: 1px solid #394858;
                border-radius: 6px;
                padding: 9px 12px;
                font-weight: 650;
            }
            QPushButton:hover {
                background: #314052;
            }
            QPushButton:disabled {
                color: #687482;
                background: #1a2028;
                border-color: #252e38;
            }
            QPushButton:checked {
                background: #375a7f;
                border-color: #69a3d8;
            }
            QPushButton#primaryButton {
                background: #1f7a4c;
                border-color: #2da765;
            }
            QPushButton#dangerButton {
                background: #8c2f39;
                border-color: #c24b56;
            }
            QLineEdit, QSpinBox, QDoubleSpinBox, QTextEdit {
                background: #0d1116;
                border: 1px solid #2b3440;
                border-radius: 6px;
                padding: 6px;
            }
            QTextEdit#logsText {
                background: #0b0f13;
                color: #cbd5df;
            }
            QLabel[state="green"], QLabel[complete="true"] {
                color: #74d99f;
            }
            QLabel[state="yellow"] {
                color: #f4c95d;
            }
            QLabel[state="red"], QLabel[complete="false"] {
                color: #ff7b86;
            }
            QLabel[state] {
                background: #101419;
                border: 1px solid #2b3440;
                border-radius: 6px;
                padding: 7px 10px;
                font-weight: 700;
            }
            """
        )
