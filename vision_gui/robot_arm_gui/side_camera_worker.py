from __future__ import annotations

import time
from typing import Any

from PySide6.QtCore import QThread, Signal


def open_side_camera_capture(stream_url: str) -> Any:
    import cv2

    if stream_url.startswith(("http://", "https://")) and hasattr(cv2, "CAP_FFMPEG"):
        return cv2.VideoCapture(stream_url, cv2.CAP_FFMPEG)
    return cv2.VideoCapture(stream_url)


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
        self._running = True
        cap = None
        consecutive_failures = 0
        last_reconnect_notice = 0.0
        try:
            while self._running:
                if cap is None or not cap.isOpened():
                    if cap is not None:
                        cap.release()
                    cap = open_side_camera_capture(self.stream_url)
                    if not cap.isOpened():
                        now = time.monotonic()
                        if now - last_reconnect_notice > 2.0:
                            self.camera_status.emit(False, "Unable to open side camera stream; retrying")
                            last_reconnect_notice = now
                        time.sleep(0.5)
                        continue
                    consecutive_failures = 0
                    self.camera_status.emit(True, "Side camera running")

                ok, frame = cap.read()
                if not ok:
                    consecutive_failures += 1
                    if consecutive_failures >= 5:
                        self.camera_status.emit(False, "Side camera read failed; reconnecting")
                        cap.release()
                        cap = None
                        time.sleep(0.5)
                    else:
                        time.sleep(0.1)
                    continue
                consecutive_failures = 0
                self.frame_ready.emit(frame)
                elapsed = time.monotonic() - self._last_frame_at
                self._last_frame_at = time.monotonic()
                if elapsed < 0.03:
                    time.sleep(0.03 - elapsed)
        finally:
            if cap is not None:
                cap.release()
            self.camera_status.emit(False, "Side camera stopped")
