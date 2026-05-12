from __future__ import annotations

import time
from typing import Any

from PySide6.QtCore import QThread, Signal


class SideCameraWorker(QThread):
    frame_ready = Signal(object)
    camera_status = Signal(bool, str)

    def __init__(self, stream_url: str, parent: Any = None) -> None:
        super().__init__(parent)
        self.stream_url = stream_url
        self._running = False
        self._last_frame_at = 0.0

    def stop(self) -> None:
        self._running = False
        self.wait(1200)

    def run(self) -> None:
        import cv2

        self._running = True
        cap = cv2.VideoCapture(self.stream_url)
        if not cap.isOpened():
            self.camera_status.emit(False, "Unable to open side camera stream")
            self._running = False
            return

        self.camera_status.emit(True, "Side camera running")
        try:
            while self._running:
                ok, frame = cap.read()
                if not ok:
                    self.camera_status.emit(False, "Side camera read failed")
                    time.sleep(0.15)
                    continue
                self.frame_ready.emit(frame)
                elapsed = time.monotonic() - self._last_frame_at
                self._last_frame_at = time.monotonic()
                if elapsed < 0.03:
                    time.sleep(0.03 - elapsed)
        finally:
            cap.release()
            self.camera_status.emit(False, "Side camera stopped")
