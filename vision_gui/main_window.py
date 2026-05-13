from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Any

import numpy as np
from PySide6.QtCore import QObject, QRunnable, QThreadPool, QTimer, Qt, Signal, Slot
from PySide6.QtWidgets import (
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QSizePolicy,
    QStackedWidget,
    QStatusBar,
    QToolBar,
    QVBoxLayout,
    QWidget,
)

from .camera_thread import CameraThread
from .client import PiClient
from .detect import HsvRange, detect_object, draw_overlay, pixel_to_robot
from .pages.calibrate import CalibratePage
from .pages.camera import CameraPage
from .pages.pick import PickPage
from .pages.setup import SetupPage

logger = logging.getLogger(__name__)

_SETTINGS_PATH = Path.home() / ".config" / "robot-arm-gui" / "settings.json"


def _load_settings() -> dict[str, Any]:
    try:
        return json.loads(_SETTINGS_PATH.read_text())
    except Exception:
        return {}


def _save_settings(s: dict[str, Any]) -> None:
    _SETTINGS_PATH.parent.mkdir(parents=True, exist_ok=True)
    _SETTINGS_PATH.write_text(json.dumps(s, indent=2))


class _Relay(QObject):
    """Carries results from worker threads back to the main thread via Qt signals."""
    done = Signal(object)
    error = Signal(str)


class _Task(QRunnable):
    """Fire-and-forget background task. Uses Qt signals for thread-safe UI callbacks."""

    def __init__(self, fn, on_done=None, on_error=None) -> None:
        super().__init__()
        self._fn = fn
        self._relay = _Relay()
        if on_done:
            self._relay.done.connect(on_done, Qt.ConnectionType.QueuedConnection)
        if on_error:
            self._relay.error.connect(on_error, Qt.ConnectionType.QueuedConnection)

    @Slot()
    def run(self) -> None:
        try:
            result = self._fn()
            self._relay.done.emit(result)
        except Exception as exc:
            logger.error("Background task failed: %s", exc)
            self._relay.error.emit(str(exc))


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Robot Arm Control")
        self.resize(1100, 700)

        self._settings = _load_settings()
        self._client = PiClient(self._settings.get("piUrl", ""))
        self._pi_ok = False
        self._calibration: dict[str, Any] = {}
        self._motion_enabled = False
        self._homography: list[list[float]] | None = None
        self._hsv = HsvRange()
        self._last_frame: np.ndarray | None = None
        self._camera_thread: CameraThread | None = None

        self._build_ui()
        self._connect_signals()

        # Polling timers — only do anything when _pi_ok is True
        self._cal_timer = QTimer(self)
        self._cal_timer.setInterval(5000)
        self._cal_timer.timeout.connect(self._poll_calibration)
        self._cal_timer.start()

        self._esp_timer = QTimer(self)
        self._esp_timer.setInterval(800)
        self._esp_timer.timeout.connect(self._poll_esp)
        self._esp_timer.start()

        # Start camera after the event loop is running, not during __init__
        QTimer.singleShot(0, self._start_camera)

    # ── UI construction ───────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        toolbar = QToolBar("Navigation")
        toolbar.setMovable(False)
        self.addToolBar(toolbar)

        self._pages: dict[str, QWidget] = {}
        self._nav_btns: dict[str, QPushButton] = {}
        self._stack = QStackedWidget()

        self.setup_page = SetupPage()
        self.camera_page = CameraPage()
        self.calibrate_page = CalibratePage()
        self.pick_page = PickPage()

        for key, label, page in [
            ("setup",     "Setup",     self.setup_page),
            ("camera",    "Camera",    self.camera_page),
            ("calibrate", "Calibrate", self.calibrate_page),
            ("pick",      "Pick",      self.pick_page),
        ]:
            self._pages[key] = page
            self._stack.addWidget(page)
            btn = QPushButton(label)
            btn.setCheckable(True)
            btn.clicked.connect(lambda _=False, k=key: self._go_to(k))
            self._nav_btns[key] = btn
            toolbar.addWidget(btn)

        spacer = QWidget()
        spacer.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        toolbar.addWidget(spacer)

        self._pi_status_lbl = QLabel("Pi: disconnected")
        self._pi_status_lbl.setStyleSheet("color: #f87171; margin-right: 12px;")
        toolbar.addWidget(self._pi_status_lbl)

        self.setCentralWidget(self._stack)
        self.setStatusBar(QStatusBar())

        self.setup_page.set_url(self._settings.get("piUrl", ""))
        self._go_to("setup")

    def _connect_signals(self) -> None:
        self.setup_page.connect_requested.connect(self._on_connect)
        self.camera_page.hsv_changed.connect(self._on_hsv_changed)
        self.camera_page.camera_index_changed.connect(self._restart_camera)
        self.calibrate_page.save_homography_requested.connect(self._save_homography)
        self.calibrate_page.save_pitch_requested.connect(self._save_pitch)
        self.pick_page.hover_requested.connect(lambda: self._do_pick(hover_only=True))
        self.pick_page.pick_requested.connect(lambda: self._do_pick(hover_only=False))
        self.pick_page.enable_motion_requested.connect(self._set_motion_enabled)

    # ── navigation ────────────────────────────────────────────────────────────

    def _go_to(self, key: str) -> None:
        for k, btn in self._nav_btns.items():
            btn.setChecked(k == key)
        self._stack.setCurrentWidget(self._pages[key])

    # ── camera ────────────────────────────────────────────────────────────────

    def _start_camera(self) -> None:
        idx = self._settings.get("cameraIndex", 0)
        self._camera_thread = CameraThread(idx)
        self._camera_thread.frame_ready.connect(self._on_frame)
        self._camera_thread.error.connect(
            lambda e: self.statusBar().showMessage(f"Camera: {e}", 5000)
        )
        self._camera_thread.start()

    def _restart_camera(self, index: int) -> None:
        self._settings["cameraIndex"] = index
        _save_settings(self._settings)
        if self._camera_thread:
            self._camera_thread.stop()
        self._camera_thread = CameraThread(index)
        self._camera_thread.frame_ready.connect(self._on_frame)
        self._camera_thread.error.connect(
            lambda e: self.statusBar().showMessage(f"Camera: {e}", 5000)
        )
        self._camera_thread.start()

    @Slot(object)
    def _on_frame(self, frame: np.ndarray) -> None:
        self._last_frame = frame
        det = detect_object(frame, self._hsv)
        robot_xy = None
        if det.found and self._homography:
            robot_xy = pixel_to_robot(det.pixel_x, det.pixel_y, self._homography)

        current = self._stack.currentWidget()
        if current is self.camera_page:
            annotated = draw_overlay(frame, det, robot_xy)
            self.camera_page.update_frame(annotated)
            if det.found:
                xy = f"({robot_xy[0]:.1f}, {robot_xy[1]:.1f})" if robot_xy else "pixels only"
                self.camera_page.set_detection_label(f"Object at {xy} — conf={det.confidence:.2f}")
            else:
                self.camera_page.set_detection_label("No object detected")
        elif current is self.calibrate_page:
            self.calibrate_page.update_frame(frame)
        elif current is self.pick_page:
            self.pick_page.update_frame(frame, det, robot_xy)

    # ── connection ────────────────────────────────────────────────────────────

    def _on_connect(self, url: str) -> None:
        self._client.set_url(url)
        self._settings["piUrl"] = url
        _save_settings(self._settings)
        self.setup_page.set_status("Connecting...", ok=False)

        def task():
            import requests
            r = requests.get(f"{url.rstrip('/')}/health", timeout=4.0)
            r.raise_for_status()
            return True

        def done(_):
            self._pi_ok = True
            self.setup_page.set_status(f"Connected: {url}", ok=True)
            self._pi_status_lbl.setText("Pi: connected")
            self._pi_status_lbl.setStyleSheet("color: #4ade80; margin-right: 12px;")
            self._poll_calibration()

        def err(msg: str):
            self._pi_ok = False
            self.setup_page.set_status(f"Failed: {msg}", ok=False)
            self._pi_status_lbl.setText("Pi: disconnected")
            self._pi_status_lbl.setStyleSheet("color: #f87171; margin-right: 12px;")

        self._run(task, done, err)

    # ── calibration + config polling ──────────────────────────────────────────

    def _poll_calibration(self) -> None:
        if not self._pi_ok:
            return

        def task():
            cal = self._client.get_calibration()
            cfg = self._client.get_config()
            return cal, cfg

        def done(result):
            cal, cfg = result
            self._calibration = cal
            self._motion_enabled = bool(cfg.get("motionEnabled", False))

            hom = cal.get("homography")
            if isinstance(hom, list) and len(hom) == 3 and isinstance(hom[0], list):
                self._homography = hom
            elif isinstance(hom, list) and len(hom) == 9:
                self._homography = [hom[i * 3:(i + 1) * 3] for i in range(3)]
            else:
                self._homography = None

            self.calibrate_page.update_calibration_summary(cal)
            calibrated = (
                self._homography is not None
                and isinstance(cal.get("pickupPitchDeg"), (int, float))
            )
            self.pick_page.set_button_state(
                calibrated=calibrated,
                pi_ok=self._pi_ok,
                motion_enabled=self._motion_enabled,
            )

        def err(msg: str):
            logger.warning("Calibration poll failed: %s", msg)

        self._run(task, done, err)

    # ── ESP polling ───────────────────────────────────────────────────────────

    def _poll_esp(self) -> None:
        if not self._pi_ok:
            return

        def task():
            return self._client.esp_status()

        def done(status: dict):
            self._esp_status = status
            self.calibrate_page.update_esp_status(status)

        self._run(task, done)

    # ── HSV ───────────────────────────────────────────────────────────────────

    def _on_hsv_changed(self, hsv: HsvRange) -> None:
        self._hsv = hsv
        self._settings["hsv"] = hsv.to_dict()
        _save_settings(self._settings)

    # ── calibration saves ─────────────────────────────────────────────────────

    def _save_homography(self, homography: list, points: list) -> None:
        def task():
            return self._client.save_homography(homography, points)

        def done(_):
            self.calibrate_page.set_homography_saved(True)
            self._homography = homography
            self.statusBar().showMessage("Homography saved.", 3000)

        def err(msg: str):
            self.calibrate_page.set_homography_saved(False)
            self.statusBar().showMessage(f"Homography save failed: {msg}", 5000)

        self._run(task, done, err)

    def _save_pitch(self, pitch: float) -> None:
        def task():
            return self._client.save_pitch(pitch)

        def done(_):
            self.statusBar().showMessage(f"Pitch {pitch:.1f}° saved.", 3000)
            self._poll_calibration()

        def err(msg: str):
            self.statusBar().showMessage(f"Pitch save failed: {msg}", 5000)

        self._run(task, done, err)

    # ── pick / hover ──────────────────────────────────────────────────────────

    def _do_pick(self, hover_only: bool) -> None:
        if self._last_frame is None:
            self.pick_page.set_pick_status("No camera frame available.")
            return

        det = detect_object(self._last_frame, self._hsv)
        if not det.found or not self._homography:
            self.pick_page.set_pick_status("No object detected — cannot pick.")
            return

        robot_xy = pixel_to_robot(det.pixel_x, det.pixel_y, self._homography)
        if robot_xy is None:
            self.pick_page.set_pick_status("Pixel→robot transform failed.")
            return

        self.pick_page.set_pick_status("Sending hover..." if hover_only else "Sending pick...")

        from datetime import UTC, datetime
        target = {
            "type": "object_detected",
            "object": "green_object",
            "pixelX": det.pixel_x,
            "pixelY": det.pixel_y,
            "robotX": robot_xy[0],
            "robotY": robot_xy[1],
            "confidence": det.confidence,
            "receivedAt": datetime.now(UTC).isoformat().replace("+00:00", "Z"),
        }

        def task():
            self._client.post_target(target)
            return self._client.pick(hover_only)

        def done(plan: dict):
            self.pick_page.update_plan(plan)
            if not plan.get("ok"):
                errors = "; ".join(plan.get("errors", ["unknown error"]))
                self.pick_page.set_pick_status(f"Blocked: {errors}")
            elif plan.get("sent"):
                self.pick_page.set_pick_status("Hovering..." if hover_only else "Picking...")
            else:
                self.pick_page.set_pick_status("Plan OK — motion disabled on Pi.")

        def err(msg: str):
            self.pick_page.set_pick_status(f"Error: {msg}")

        self._run(task, done, err)

    # ── motion enable ─────────────────────────────────────────────────────────

    def _set_motion_enabled(self, enabled: bool) -> None:
        def task():
            return self._client.patch_config({"motionEnabled": enabled})

        def done(_):
            self._motion_enabled = enabled
            self.statusBar().showMessage(
                f"Motion {'enabled' if enabled else 'disabled'}.", 2000
            )
            self._poll_calibration()

        def err(msg: str):
            self.statusBar().showMessage(f"Motion toggle failed: {msg}", 5000)

        self._run(task, done, err)

    # ── background task runner ────────────────────────────────────────────────

    def _run(self, fn, on_done=None, on_error=None) -> None:
        task = _Task(fn, on_done, on_error)
        task.setAutoDelete(True)
        QThreadPool.globalInstance().start(task)

    # ── cleanup ───────────────────────────────────────────────────────────────

    def closeEvent(self, event) -> None:
        if self._camera_thread:
            self._camera_thread.stop()
        _save_settings(self._settings)
        super().closeEvent(event)
