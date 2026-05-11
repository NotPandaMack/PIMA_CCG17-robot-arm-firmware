from __future__ import annotations

import time
from typing import Any

from PySide6.QtCore import QThread, Signal

from .detection_worker import HSVProfile, detect_green_object, draw_detection_overlay, pixel_to_robot


class CameraWorker(QThread):
    frame_ready = Signal(object)
    detection_ready = Signal(object)
    camera_status = Signal(bool, str)

    def __init__(self, camera_index: int = 0, parent: Any = None) -> None:
        super().__init__(parent)
        self.camera_index = camera_index
        self.profile = HSVProfile()
        self.homography: list[list[float]] | None = None
        self.detect_enabled = True
        self._running = False
        self._last_frame_at = 0.0

    def configure(
        self,
        *,
        camera_index: int | None = None,
        profile: HSVProfile | None = None,
        homography: list[list[float]] | None = None,
        detect_enabled: bool | None = None,
    ) -> None:
        if camera_index is not None:
            self.camera_index = int(camera_index)
        if profile is not None:
            self.profile = profile
        if homography is not None:
            self.homography = homography
        if detect_enabled is not None:
            self.detect_enabled = detect_enabled

    def stop(self) -> None:
        self._running = False
        self.wait(1200)

    def run(self) -> None:
        import cv2

        self._running = True
        cap = cv2.VideoCapture(self.camera_index)
        if not cap.isOpened():
            self.camera_status.emit(False, f"Unable to open webcam index {self.camera_index}")
            self._running = False
            return

        self.camera_status.emit(True, f"Webcam {self.camera_index} running")
        try:
            while self._running:
                ok, frame = cap.read()
                if not ok:
                    self.camera_status.emit(False, "Camera read failed")
                    time.sleep(0.1)
                    continue

                detection = detect_green_object(frame, self.profile) if self.detect_enabled else None
                robot_xy = None
                if detection and detection.found and self.homography:
                    robot_xy = pixel_to_robot(detection.pixel_x or 0, detection.pixel_y or 0, self.homography)

                overlay = draw_detection_overlay(frame, detection, robot_xy, calibrated=bool(self.homography)) if detection else frame
                self.frame_ready.emit(overlay)
                if detection:
                    self.detection_ready.emit({"detection": detection, "robot_xy": robot_xy})

                elapsed = time.monotonic() - self._last_frame_at
                self._last_frame_at = time.monotonic()
                if elapsed < 0.015:
                    time.sleep(0.015 - elapsed)
        finally:
            cap.release()
            self.camera_status.emit(False, "Camera stopped")
