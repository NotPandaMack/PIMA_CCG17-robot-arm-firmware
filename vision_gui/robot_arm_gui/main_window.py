from __future__ import annotations

import logging
import time
from copy import deepcopy
from time import perf_counter
from typing import Any

from PySide6.QtCore import QTimer, Qt, QThreadPool
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
    empty_calibration,
    has_homography,
    load_local_calibration,
    pickup_pose_calibrated,
    save_local_calibration,
    table_z_calibrated,
    workspace_bounds_saved,
)
from .config import DEFAULT_HSV_PROFILE, DEFAULT_SETTINGS, load_settings, normalize_http_url, save_settings
from .detection_worker import DetectionResult, HSVProfile
from .esp_client import EspClient
from .pi_client import PiClient
from .robot_motion_adapter import RobotMotionAdapter
from .workers import TaskWorker
from .widgets.autonomous_page import AutonomousPage
from .widgets.calibration_page import CalibrationPage
from .widgets.camera_page import CameraPage
from .widgets.connection_page import ConnectionPage
from .widgets.logs_panel import LogsPanel
from .widgets.status_widgets import ChecklistPanel, SafetyBar, StatusPill
from .widgets.test_page import TestPage


logger = logging.getLogger(__name__)


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        startup_started = perf_counter()
        logger.info("GUI startup begins")
        super().__init__()
        self.setWindowTitle("Robot Arm Control Center")
        self.thread_pool = QThreadPool.globalInstance()
        self._active_workers: list[TaskWorker] = []
        self.settings = deepcopy(DEFAULT_SETTINGS)
        self.calibration = empty_calibration()
        self.pi_client = PiClient(self.settings["piUrl"], mock=bool(self.settings.get("mockPi")))
        self.esp_client = EspClient(self.settings["espUrl"], fake=bool(self.settings.get("fakeEsp")))

        self.pi_connected = False
        self.esp_connected = False
        self.webcam_connected = False
        self.pi_tested = False
        self.esp_tested = False
        self.webcam_tested = False
        self.last_esp_status: dict[str, Any] | None = None
        self.last_detection: DetectionResult | None = None
        self.last_robot_xy: tuple[float, float] | None = None
        self.last_plan: dict[str, Any] | None = None
        self.last_frame_size: tuple[int, int] | None = None
        self.last_target_status = "none"
        self.last_send_at = 0.0
        self.camera_worker: Any | None = None

        self._build_ui()
        self._connect_signals()
        self._apply_settings_to_ui()
        self._apply_style()
        self._start_nonblocking_timers()
        self.go_to("setup")
        self.update_ui_state()
        QTimer.singleShot(0, self._load_startup_files_async)
        elapsed_ms = (perf_counter() - startup_started) * 1000.0
        logger.info("GUI startup ends in %.1f ms", elapsed_ms)
        if elapsed_ms > 100.0:
            logger.warning("Startup widget construction took %.1f ms", elapsed_ms)

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

        self.calibration_page.camera_view.mapped_clicked.connect(self.handle_calibration_click)
        self.calibration_page.grid_overlay_changed.connect(self.update_calibration_grid_overlay)
        self.calibration_page.save_touch_requested.connect(self.save_table_touch_point)
        self.calibration_page.save_pickup_requested.connect(self.save_pickup_pose_from_esp)
        self.calibration_page.save_step_requested.connect(self.save_calibration_step)
        self.calibration_page.preview_hover_requested.connect(lambda: self.generate_preview(True))
        self.calibration_page.test_hover_requested.connect(lambda: self.run_pick(True))
        self.calibration_page.finish_requested.connect(self.finish_calibration)

        self.test_page.command_requested.connect(self.send_esp_command)
        self.test_page.jog_requested.connect(self.send_safe_jog)
        self.test_page.refresh_requested.connect(self.refresh_status)
        self.test_page.enable_motion_requested.connect(self.enable_motion)
        self.test_page.dry_run_changed.connect(self.set_movement_adapter_dry_run)

        self.autonomous_page.preview_requested.connect(self.generate_preview)
        self.autonomous_page.pick_requested.connect(self.run_pick)
        self.autonomous_page.auto_pick_changed.connect(self.set_auto_pick_enabled)

    def _apply_settings_to_ui(self) -> None:
        self.connection_page.set_from_settings(self.settings)
        profile = HSVProfile.from_settings(self.settings.get("hsv", DEFAULT_HSV_PROFILE))
        self.camera_page.set_profile(profile)
        self.camera_page.continuous.setChecked(bool(self.settings.get("continuousSend", False)))
        self.camera_page.rate.setValue(int(float(self.settings.get("sendRateHz", 5.0))))
        self.test_page.set_dry_run(bool(self.settings.get("movementAdapterDryRun", True)))

    def _start_nonblocking_timers(self) -> None:
        self.auto_pick_timer = QTimer(self)
        self.auto_pick_timer.timeout.connect(self._auto_pick_tick)
        self.auto_pick_timer.start(500)
        self._stable_since: float | None = None
        self._last_auto_pick_at = 0.0

    def _load_startup_files_async(self) -> None:
        self.log("Startup safe mode: Setup is ready. Hardware has not been tested yet.")

        def task() -> dict[str, Any]:
            started = perf_counter()
            settings = load_settings()
            calibration = load_local_calibration()
            elapsed_ms = (perf_counter() - started) * 1000.0
            return {"settings": settings, "calibration": calibration, "elapsedMs": elapsed_ms}

        self._run_background(task, self._on_startup_files_loaded, "Startup settings load failed")

    def _on_startup_files_loaded(self, result: dict[str, Any]) -> None:
        elapsed_ms = float(result.get("elapsedMs", 0.0))
        if elapsed_ms > 100.0:
            logger.warning("Startup settings/calibration load took %.1f ms", elapsed_ms)
            self.log(f"Startup warning: local settings load took {elapsed_ms:.0f} ms.")
        self.settings = result["settings"]
        self.calibration = result["calibration"]
        self.pi_client.set_base_url(self.settings["piUrl"])
        self.pi_client.set_mock(bool(self.settings.get("mockPi")))
        self.esp_client.set_base_url(self.settings["espUrl"])
        self.esp_client.set_fake(bool(self.settings.get("fakeEsp")))
        self._apply_settings_to_ui()
        self.log("Saved settings loaded. Status remains Not tested until you press a test button.")
        self.update_ui_state()

    def _run_background(self, task: Any, on_result: Any, error_prefix: str, on_error: Any | None = None) -> None:
        worker = TaskWorker(task)
        self._active_workers.append(worker)
        worker.signals.result.connect(on_result)
        def handle_error(error: str, prefix: str = error_prefix) -> None:
            message = error.splitlines()[0]
            self.log(f"{prefix}: {message}")
            if on_error is not None:
                on_error(message)

        worker.signals.error.connect(handle_error)
        worker.signals.finished.connect(lambda w=worker: self._active_workers.remove(w) if w in self._active_workers else None)
        self.thread_pool.start(worker)

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
        self.update_ui_state()

    def test_pi_connection(self) -> None:
        self.save_setup_settings()
        self.pi_tested = True
        self.connection_page.pi_status.set_state("Pi testing", "yellow")
        self.log("Testing Pi connection in the background.")

        pi_url = self.settings["piUrl"]
        mock = bool(self.settings.get("mockPi"))

        def task() -> dict[str, Any]:
            client = PiClient(pi_url, mock=mock)
            client.health()
            config = client.get_config().get("config", {})
            calibration = client.get_calibration()
            return {"config": config, "calibration": calibration}

        self._run_background(task, self._on_pi_test_result, "Pi server unreachable", self._on_pi_offline)

    def _on_pi_offline(self, _message: str) -> None:
        self.pi_connected = False
        self.update_ui_state()

    def _on_pi_test_result(self, result: dict[str, Any]) -> None:
        self.pi_connected = True
        self.log("Pi server connected.")
        self._apply_pi_config_and_calibration(result.get("config", {}), result.get("calibration", {}))
        self.update_ui_state()

    def test_esp_connection(self) -> None:
        self.save_setup_settings()
        self.esp_tested = True
        self.connection_page.esp_status.set_state("ESP testing", "yellow")
        self.log("Testing ESP connection in the background.")

        esp_url = self.settings["espUrl"]
        fake = bool(self.settings.get("fakeEsp"))

        def task() -> dict[str, Any]:
            return EspClient(esp_url, fake=fake).status()

        self._run_background(task, self._on_esp_status_result, "ESP unreachable", self._on_esp_offline)

    def _on_esp_offline(self, _message: str) -> None:
        self.esp_connected = False
        self.last_esp_status = None
        self.update_ui_state()

    def _on_esp_status_result(self, status: dict[str, Any]) -> None:
        self.last_esp_status = status
        self.esp_connected = True
        self.log("ESP connected.")
        self.update_ui_state()

    def test_webcam(self) -> None:
        self.webcam_tested = True
        index = int(self.settings.get("cameraIndex", 0))
        self.connection_page.camera_status.set_state("Camera testing", "yellow")
        self.log(f"Testing webcam {index} in the background.")

        def task() -> dict[str, Any]:
            import cv2

            cap = cv2.VideoCapture(index)
            try:
                ok = cap.isOpened()
                frame_size = None
                if ok:
                    frame_ok, frame = cap.read()
                    ok = bool(frame_ok)
                    if frame_ok:
                        frame_size = (frame.shape[1], frame.shape[0])
                return {"ok": ok, "frameSize": frame_size}
            finally:
                cap.release()

        self._run_background(task, lambda result: self._on_webcam_test_result(index, result), "Webcam test failed", self._on_webcam_offline)

    def _on_webcam_offline(self, _message: str) -> None:
        self.webcam_connected = False
        self.update_ui_state()

    def _on_webcam_test_result(self, index: int, result: dict[str, Any]) -> None:
        self.webcam_connected = bool(result.get("ok"))
        self.last_frame_size = result.get("frameSize")
        self.log(f"Webcam {index} {'connected' if self.webcam_connected else 'failed to open'}.")
        self.update_ui_state()

    def auto_detect_pi(self) -> None:
        self.pi_tested = True
        candidates = [self.settings.get("piUrl", ""), "http://raspberrypi.local:8000"]
        self.connection_page.pi_status.set_state("Pi auto-detecting", "yellow")
        self.log("Auto-detecting Pi server in the background.")

        def task() -> str | None:
            for candidate in candidates:
                if not candidate:
                    continue
                try:
                    PiClient(candidate).health()
                    return candidate
                except Exception:
                    continue
            return None

        self._run_background(task, self._on_auto_detect_result, "Auto-detect failed")

    def _on_auto_detect_result(self, candidate: str | None) -> None:
        if candidate:
            self.settings["piUrl"] = candidate
            self.pi_client.set_base_url(candidate)
            self.connection_page.set_from_settings(self.settings)
            self.pi_connected = True
            self.log(f"Auto-detected Pi server at {candidate}.")
        else:
            self.pi_connected = False
            self.log("Auto-detect did not find a Pi server. Enter the Pi URL manually.")
        self.update_ui_state()

    def refresh_status(self) -> None:
        self.log("Refreshing status in the background.")
        self.pi_tested = True
        self.esp_tested = True
        pi_url = self.settings["piUrl"]
        esp_url = self.settings["espUrl"]
        mock_pi = bool(self.settings.get("mockPi"))
        fake_esp = bool(self.settings.get("fakeEsp"))

        def task() -> dict[str, Any]:
            result: dict[str, Any] = {"piConnected": False, "espConnected": False, "espStatus": None, "config": {}, "calibration": {}}
            try:
                pi = PiClient(pi_url, mock=mock_pi)
                pi.health()
                result["piConnected"] = True
                result["config"] = pi.get_config().get("config", {})
                result["calibration"] = pi.get_calibration()
            except Exception as error:
                result["piError"] = str(error)
            try:
                result["espStatus"] = EspClient(esp_url, fake=fake_esp).status()
                result["espConnected"] = True
            except Exception as error:
                result["espError"] = str(error)
            return result

        self._run_background(task, self._on_refresh_result, "Status refresh failed")

    def _on_refresh_result(self, result: dict[str, Any]) -> None:
        self.pi_connected = bool(result.get("piConnected"))
        self.esp_connected = bool(result.get("espConnected"))
        self.last_esp_status = result.get("espStatus")
        self._apply_pi_config_and_calibration(result.get("config", {}), result.get("calibration", {}))
        if result.get("piError"):
            self.log(f"Pi offline: {result['piError']}")
        if result.get("espError"):
            self.log(f"ESP offline: {result['espError']}")
        self.update_ui_state()

    def _apply_pi_config_and_calibration(self, config: dict[str, Any], calibration: dict[str, Any]) -> None:
        if isinstance(config, dict) and config:
            self.settings["motionEnabled"] = bool(config.get("motionEnabled", self.settings.get("motionEnabled", False)))
            self.settings["workspace"] = config.get("workspace", self.settings.get("workspace", {}))
            if config.get("espBaseUrl") and "ESP8266_IP" not in str(config.get("espBaseUrl")):
                self.settings["espUrl"] = config["espBaseUrl"]
                self.esp_client.set_base_url(config["espBaseUrl"])
        if isinstance(calibration, dict) and calibration.get("homography"):
            self.calibration.update(calibration)

    def start_camera(self) -> None:
        if self.camera_worker and self.camera_worker.isRunning():
            return
        from .camera_worker import CameraWorker

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
        self.webcam_tested = True
        self.webcam_connected = online
        self.log(message)
        self.update_ui_state()

    def on_camera_frame(self, frame: Any) -> None:
        h, w = frame.shape[:2]
        self.last_frame_size = (w, h)
        image = QImage(frame.data, w, h, frame.strides[0], QImage.Format_BGR888).copy()
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
        pi_url = self.settings["piUrl"]
        mock = bool(self.settings.get("mockPi"))

        def task() -> dict[str, Any]:
            return PiClient(pi_url, mock=mock).post_target(payload)

        def on_result(result: dict[str, Any]) -> None:
            self.last_send_at = time.monotonic()
            target = result.get("target", payload)
            self.last_target_status = "valid" if target.get("robotX") is not None else "pixels"
            if not silent:
                self.log("Detected target sent to Pi.")
            self.update_ui_state()

        self._run_background(task, on_result, "Target not sent")

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

    def handle_calibration_click(self, mapped: dict[str, Any]) -> None:
        x = int(mapped["pixelX"])
        y = int(mapped["pixelY"])
        self.calibration_page.capture_click(x, y, mapped)
        if self.calibration_page.step_index == 6:
            robot_xy = convert_pixel(self.calibration, x, y)
            if robot_xy:
                marker_text = (
                    f"widget=({mapped.get('widgetX', 0):.0f},{mapped.get('widgetY', 0):.0f}) "
                    f"pixel=({x},{y}) robot=({robot_xy[0]:.1f},{robot_xy[1]:.1f})"
                )
                self.calibration_page.camera_view.set_marker(x, y, marker_text)
                self.calibration_page.set_validation_result(
                    "Visual test mode\n"
                    f"pixelX/pixelY: {x}, {y}\n"
                    f"robotX/robotY: {robot_xy[0]:.1f}, {robot_xy[1]:.1f}\n"
                    f"raw widget click: {mapped.get('widgetX', 0):.1f}, {mapped.get('widgetY', 0):.1f}\n"
                    f"displayed image click: {mapped.get('displayX', 0):.1f}, {mapped.get('displayY', 0):.1f}"
                )
                payload = {
                    "type": "object_detected",
                    "object": "calibration_validation",
                    "pixelX": x,
                    "pixelY": y,
                    "robotX": robot_xy[0],
                    "robotY": robot_xy[1],
                    "confidence": 1.0,
                }
                pi_url = self.settings["piUrl"]
                mock = bool(self.settings.get("mockPi"))

                def task() -> dict[str, Any]:
                    return PiClient(pi_url, mock=mock).post_target(payload)

                self._run_background(task, lambda _result: self.log("Calibration validation target sent to Pi."), "Validation target could not be sent")
            else:
                self.calibration_page.camera_view.set_marker(x, y, f"px=({x},{y})")
                self.calibration_page.set_validation_result("Conversion unavailable. Complete camera mapping first.")

    def update_calibration_grid_overlay(self, enabled: bool) -> None:
        self.calibration_page.camera_view.set_grid_overlay(
            enabled,
            self.calibration.get("homography") if has_homography(self.calibration) else None,
            self.calibration.get("workspaceBounds", self.settings.get("workspace", {})),
        )

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
            pi_url = self.settings["piUrl"]
            mock = bool(self.settings.get("mockPi"))

            def task() -> dict[str, Any]:
                return PiClient(pi_url, mock=mock).put_config({"workspace": workspace})

            self._run_background(task, lambda _result: self.log("Workspace bounds saved to Pi."), "Workspace saved locally but not uploaded to Pi")
        elif step == 4:
            self.calibration["tableZ"] = self.calibration_page.table_z()
            self.log("Table Z step saved.")
        elif step == 5:
            self.calibration.update(self.calibration_page.pickup_values())
            self.log("Pickup pose values saved.")
        self.update_ui_state()

    def save_table_touch_point(self, label: str) -> None:
        esp_url = self.settings["espUrl"]
        fake = bool(self.settings.get("fakeEsp"))

        def task() -> dict[str, Any]:
            return EspClient(esp_url, fake=fake).status()

        def on_result(status: dict[str, Any]) -> None:
            self.last_esp_status = status
            self.esp_connected = True
            self.calibration_page.add_table_point(label, status)
            self.calibration["tableZ"] = self.calibration_page.table_z()
            self.log(f"Saved table touch point: {label}.")
            self.update_ui_state()

        self._run_background(task, on_result, "Touch point not saved")

    def save_pickup_pose_from_esp(self) -> None:
        esp_url = self.settings["espUrl"]
        fake = bool(self.settings.get("fakeEsp"))

        def task() -> dict[str, Any]:
            return EspClient(esp_url, fake=fake).status()

        def on_result(status: dict[str, Any]) -> None:
            self.last_esp_status = status
            self.esp_connected = True
            if "pitch" not in status or "z" not in status:
                self.log("Pickup pose blocked: ESP status does not include pitch and Z.")
                return
            self.calibration_page.set_pickup_from_status(status)
            self.calibration.update(self.calibration_page.pickup_values())
            self.log("Pickup pose captured from ESP status.")
            self.update_ui_state()

        self._run_background(task, on_result, "Pickup pose not saved")

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
        pi_url = self.settings["piUrl"]
        mock = bool(self.settings.get("mockPi"))
        calibration_payload = dict(self.calibration)

        def task() -> dict[str, Any]:
            client = PiClient(pi_url, mock=mock)
            client.put_calibration(calibration_payload)
            return client.put_config(config_patch)

        self._run_background(task, lambda _result: self.log("Calibration saved locally and uploaded to Pi."), "Calibration saved locally but Pi upload failed")
        if self.camera_worker:
            self.camera_worker.configure(homography=self.calibration.get("homography"))
        self.update_ui_state()

    def generate_preview(self, hover_only: bool = False) -> None:
        if not self._preview_gate():
            return
        payload = self.last_detection.as_payload(self.last_robot_xy) if self.last_detection else None
        pi_url = self.settings["piUrl"]
        mock = bool(self.settings.get("mockPi"))
        self.log("Generating pickup preview in the background.")

        def task() -> dict[str, Any]:
            client = PiClient(pi_url, mock=mock)
            if payload is not None:
                client.post_target(payload)
            return client.preview_pick(hover_only)

        def on_result(plan: dict[str, Any]) -> None:
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
            self.update_ui_state()

        self._run_background(task, on_result, "Preview failed")

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

        pi_url = self.settings["piUrl"]
        mock = bool(self.settings.get("mockPi"))
        self.log("Sending pickup request in the background.")

        def task() -> dict[str, Any]:
            return PiClient(pi_url, mock=mock).pick(hover_only)

        def on_result(plan: dict[str, Any]) -> None:
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
            save_settings(self.settings)
            self.update_ui_state()

        self._run_background(task, on_result, "Pickup failed")

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
        pi_url = self.settings["piUrl"]
        esp_url = self.settings["espUrl"]
        mock = bool(self.settings.get("mockPi"))

        def task() -> dict[str, Any]:
            return PiClient(pi_url, mock=mock).put_config({"motionEnabled": True, "espBaseUrl": esp_url})

        def on_result(response: dict[str, Any]) -> None:
            self.settings["motionEnabled"] = bool(response.get("config", {}).get("motionEnabled", True))
            save_settings(self.settings)
            self.log("Motion enabled. Run hover-only movement before full pickup.")
            self.update_ui_state()

        self._run_background(task, on_result, "Motion enable failed")

    def set_movement_adapter_dry_run(self, enabled: bool) -> None:
        self.settings["movementAdapterDryRun"] = enabled
        save_settings(self.settings)
        self.log("Movement adapter dry-run enabled." if enabled else "Movement adapter dry-run disabled.")

    def _movement_adapter(self, client: EspClient) -> RobotMotionAdapter:
        return RobotMotionAdapter(
            client,
            dry_run=bool(self.settings.get("movementAdapterDryRun", True)),
            logger=self.log,
        )

    def send_esp_command(self, command: str) -> None:
        if command not in {"STOP", "ESTOP", "CLEAR_ESTOP", "CLEAR_TIMELINE"} and not self.esp_connected and not self.settings.get("fakeEsp"):
            self.log(f"Command not sent: ESP is not connected ({command}).")
            return
        esp_url = self.settings["espUrl"]
        fake = bool(self.settings.get("fakeEsp"))
        dry_run = bool(self.settings.get("movementAdapterDryRun", True))

        def task() -> dict[str, Any]:
            client = EspClient(esp_url, fake=fake)
            adapter = RobotMotionAdapter(client, dry_run=dry_run, logger=lambda message: None)
            result = adapter.send_web_command(command)
            return {"result": result, "status": result.status, "command": result.command, "sent": result.sent}

        def on_result(result: dict[str, Any]) -> None:
            self.log(f"PyGUI sending same as web UI: {result['command']}")
            if not result.get("sent"):
                self.log(f"Movement adapter dry-run: not sent: {result['command']}")
            else:
                self.log(f"ESP command sent: {result['command']}")
                self._on_esp_status_result(result.get("status", {}))

        self._run_background(task, on_result, "ESP command failed")

    def send_safe_jog(self, axis: str, direction: int) -> None:
        if not self.esp_connected and not self.settings.get("fakeEsp"):
            self.log("Jog refused: ESP is not connected. Refresh or test ESP status first.")
            return
        esp_url = self.settings["espUrl"]
        fake = bool(self.settings.get("fakeEsp"))
        table_z = self._current_table_z_for_manual_jog()
        calibration_touch_mode = self.test_page.calibration_touch_mode_enabled()
        dry_run = bool(self.settings.get("movementAdapterDryRun", True))
        delta = 2.0 * direction

        def task() -> dict[str, Any]:
            client = EspClient(esp_url, fake=fake)
            adapter = RobotMotionAdapter(client, dry_run=dry_run, logger=lambda message: None)
            result = adapter.jog_axis(
                axis,
                delta,
                table_z=table_z,
                calibration_touch_mode=calibration_touch_mode,
            )
            return {"status": result.status, "pose": result.pose, "command": result.command, "sent": result.sent}

        def on_result(result: dict[str, Any]) -> None:
            pose = result["pose"]
            command = result["command"]
            self.test_page.set_proposed_jog({"x": pose.x, "y": pose.y, "z": pose.z, "pitch": pose.pitch}, command)
            self.log(f"Jog command: {command}")
            self.log(f"PyGUI sending same as web UI: {command}")
            if not result.get("sent"):
                self.log(f"Movement adapter dry-run: not sent: {command}")
            else:
                self._on_esp_status_result(result.get("status", {}))

        self._run_background(task, on_result, "Jog refused")

    def _current_table_z_for_manual_jog(self) -> float | None:
        if not self.last_esp_status:
            return None
        try:
            from .calibration_manager import estimate_table_z

            x = float(self.last_esp_status.get("x"))
            y = float(self.last_esp_status.get("y"))
            return estimate_table_z(self.calibration, x, y)
        except Exception:
            return None

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
        if estop_active is None:
            self.health_pills["estop"].set_state("ESTOP unknown", "yellow")
        else:
            self.health_pills["estop"].set_state("ESTOP active" if estop_active else "ESTOP inactive", "red" if estop_active else "green")
        self.health_pills["camera"].set_state("Webcam online" if state.webcam_connected else "Webcam offline", "green" if state.webcam_connected else "red")
        self.health_pills["calibration"].set_state("Calibrated" if calibrated else "Not calibrated", "green" if calibrated else "red")
        self.health_pills["motion"].set_state("Motion enabled" if state.motion_enabled else "Motion disabled", "green" if state.motion_enabled else "yellow")
        self.health_pills["target"].set_state(f"Target {self.last_target_status}", "green" if state.target_valid else "yellow")
        if self.calibration_page.grid_overlay.isChecked():
            self.update_calibration_grid_overlay(True)
        if not self.pi_tested:
            self.connection_page.pi_status.set_state("Pi not tested", "yellow")
            self.health_pills["pi"].set_state("Pi not tested", "yellow")
        if not self.esp_tested:
            self.connection_page.esp_status.set_state("ESP not tested", "yellow")
            self.health_pills["esp"].set_state("ESP not tested", "yellow")
            self.health_pills["estop"].set_state("ESTOP not tested", "yellow")
        if not self.webcam_tested and not self.webcam_connected:
            self.connection_page.camera_status.set_state("Camera not tested", "yellow")
            self.health_pills["camera"].set_state("Webcam not tested", "yellow")

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
