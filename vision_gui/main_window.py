from __future__ import annotations

import json
import logging
import queue
import threading
from pathlib import Path
from typing import Any

import numpy as np
from PySide6.QtCore import QTimer, Qt
from PySide6.QtWidgets import (
    QLabel,
    QMainWindow,
    QPushButton,
    QSizePolicy,
    QStackedWidget,
    QStatusBar,
    QToolBar,
    QWidget,
)

from .camera_feed import CameraFeed
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


# Thread-safe queue — background threads post callables here;
# the main thread drains it via a QTimer (see MainWindow._cb_timer).
_cb_queue: queue.Queue = queue.Queue()


def _run_bg(fn, on_done=None, on_error=None) -> None:
    """Run fn() on a daemon thread. Deliver result/error on the main thread
    via _cb_queue, which is drained by MainWindow's _cb_timer QTimer."""
    def worker():
        try:
            result = fn()
            if on_done:
                _cb_queue.put(lambda: on_done(result))
        except Exception as exc:
            logger.error("Background task failed: %s", exc)
            if on_error:
                msg = str(exc)
                _cb_queue.put(lambda: on_error(msg))
    threading.Thread(target=worker, daemon=True).start()


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

        # Camera runs on the main thread via QTimer — no thread/segfault issues
        self._camera = CameraFeed()
        self._camera.frame_ready.connect(self._on_frame)
        self._camera.error.connect(self._on_camera_error)

        self._build_ui()
        self._connect_signals()

        # Drain the background-thread callback queue on the main thread every 20 ms.
        self._cb_timer = QTimer(self)
        self._cb_timer.setInterval(20)
        self._cb_timer.timeout.connect(self._drain_callbacks)
        self._cb_timer.start()

        # Polling timers — guarded by _pi_ok, only fire network calls when connected
        self._cal_timer = QTimer(self)
        self._cal_timer.setInterval(5000)
        self._cal_timer.timeout.connect(self._poll_calibration)
        self._cal_timer.start()

        self._esp_timer = QTimer(self)
        self._esp_timer.setInterval(1000)
        self._esp_timer.timeout.connect(self._poll_esp)
        self._esp_timer.start()

    # ── UI ────────────────────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        toolbar = QToolBar()
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

        self._pi_pill = QLabel("Pi: disconnected")
        self._pi_pill.setStyleSheet("color: #f87171; margin-right: 12px;")
        toolbar.addWidget(self._pi_pill)

        self.setCentralWidget(self._stack)
        self.setStatusBar(QStatusBar())

        self.setup_page.set_url(self._settings.get("piUrl", ""))
        self.setup_page.set_stream_url(str(self._settings.get("streamUrl", "")))
        self._go_to("setup")

    def _connect_signals(self) -> None:
        self.setup_page.connect_requested.connect(self._on_connect)
        self.setup_page.stream_requested.connect(self._on_stream_connect)
        self.camera_page.hsv_changed.connect(self._on_hsv_changed)
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

    def _on_stream_connect(self, source: int | str) -> None:
        if isinstance(source, str) and not source.strip():
            self.statusBar().showMessage("Enter a stream URL or local camera index.", 3000)
            return
        self._settings["streamUrl"] = str(source)
        _save_settings(self._settings)
        self._camera.set_source(source)
        self.statusBar().showMessage(f"Opening camera: {source}", 3000)

    def _on_camera_error(self, msg: str) -> None:
        self.statusBar().showMessage(f"Camera: {msg}", 8000)
        logger.warning("Camera error: %s", msg)

    def _on_frame(self, frame: np.ndarray) -> None:
        self._last_frame = frame
        det = detect_object(frame, self._hsv)
        robot_xy = None
        if det.found and self._homography:
            robot_xy = pixel_to_robot(det.pixel_x, det.pixel_y, self._homography)

        current = self._stack.currentWidget()
        if current is self.camera_page:
            self.camera_page.update_frame(draw_overlay(frame, det, robot_xy))
            if det.found:
                xy = f"({robot_xy[0]:.1f}, {robot_xy[1]:.1f})" if robot_xy else "pixels only"
                self.camera_page.set_detection_label(f"Object at {xy} — conf={det.confidence:.2f}")
            else:
                self.camera_page.set_detection_label("No object detected")
        elif current is self.calibrate_page:
            self.calibrate_page.update_frame(frame)
        elif current is self.pick_page:
            self.pick_page.update_frame(frame, det, robot_xy)

    # ── Pi connection ─────────────────────────────────────────────────────────

    def _on_connect(self, url: str) -> None:
        if not url:
            return
        self._client.set_url(url)
        self._settings["piUrl"] = url
        _save_settings(self._settings)
        self.setup_page.set_status("Connecting...", ok=False)

        def task():
            import requests as _req
            r = _req.get(f"{url.rstrip('/')}/health", timeout=4.0)
            r.raise_for_status()
            return True

        def done(_):
            self._pi_ok = True
            self.setup_page.set_status(f"Connected: {url}", ok=True)
            self._pi_pill.setText("Pi: connected")
            self._pi_pill.setStyleSheet("color: #4ade80; margin-right: 12px;")
            self._poll_calibration()

        def err(msg: str):
            self._pi_ok = False
            self.setup_page.set_status(f"Failed: {msg}", ok=False)

        _run_bg(task, done, err)

    # ── calibration + config polling ──────────────────────────────────────────

    def _poll_calibration(self) -> None:
        if not self._pi_ok:
            return

        def task():
            return self._client.get_calibration(), self._client.get_config()

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

        _run_bg(task, done, lambda e: logger.warning("Cal poll failed: %s", e))

    # ── ESP polling ───────────────────────────────────────────────────────────

    def _poll_esp(self) -> None:
        if not self._pi_ok:
            return

        def task():
            return self._client.esp_status()

        def done(s: dict):
            self.calibrate_page.update_esp_status(s)

        _run_bg(task, done)

    # ── HSV ───────────────────────────────────────────────────────────────────

    def _on_hsv_changed(self, hsv: HsvRange) -> None:
        self._hsv = hsv
        self._settings["hsv"] = hsv.to_dict()
        _save_settings(self._settings)

    # ── saves ─────────────────────────────────────────────────────────────────

    def _save_homography(self, homography: list, points: list) -> None:
        def task():
            return self._client.save_homography(homography, points)

        def done(_):
            self.calibrate_page.set_homography_saved(True)
            self._homography = homography
            self.statusBar().showMessage("Homography saved.", 3000)

        def err(e: str):
            self.calibrate_page.set_homography_saved(False)
            self.statusBar().showMessage(f"Homography save failed: {e}", 5000)

        _run_bg(task, done, err)

    def _save_pitch(self, pitch: float) -> None:
        def task():
            return self._client.save_pitch(pitch)

        _run_bg(
            task,
            lambda _: (self.statusBar().showMessage(f"Pitch {pitch:.1f}° saved.", 3000),
                       self._poll_calibration()),
            lambda e: self.statusBar().showMessage(f"Pitch save failed: {e}", 5000),
        )

    # ── pick / hover ──────────────────────────────────────────────────────────

    def _do_pick(self, hover_only: bool) -> None:
        if self._last_frame is None:
            self.pick_page.set_pick_status("No camera frame — connect camera first.")
            return
        det = detect_object(self._last_frame, self._hsv)
        if not det.found or not self._homography:
            self.pick_page.set_pick_status("No object detected in current frame.")
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
                self.pick_page.set_pick_status("Blocked: " + "; ".join(plan.get("errors", [])))
            elif plan.get("sent"):
                self.pick_page.set_pick_status("Hovering..." if hover_only else "Picking!")
            else:
                self.pick_page.set_pick_status("Plan OK — motion disabled on Pi.")

        _run_bg(task, done, lambda e: self.pick_page.set_pick_status(f"Error: {e}"))

    # ── motion ────────────────────────────────────────────────────────────────

    def _set_motion_enabled(self, enabled: bool) -> None:
        def task():
            return self._client.patch_config({"motionEnabled": enabled})

        def done(_):
            self._motion_enabled = enabled
            self.statusBar().showMessage(
                f"Motion {'ENABLED' if enabled else 'disabled'}.", 3000)
            self._poll_calibration()

        _run_bg(task, done,
                lambda e: self.statusBar().showMessage(f"Motion toggle failed: {e}", 5000))

    # ── callback drain ────────────────────────────────────────────────────────

    def _drain_callbacks(self) -> None:
        while not _cb_queue.empty():
            try:
                _cb_queue.get_nowait()()
            except Exception as exc:
                logger.error("Callback error: %s", exc)

    # ── cleanup ───────────────────────────────────────────────────────────────

    def closeEvent(self, event) -> None:
        self._camera.stop()
        _save_settings(self._settings)
        super().closeEvent(event)
