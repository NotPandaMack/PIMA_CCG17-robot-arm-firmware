from __future__ import annotations

import logging

import cv2
import numpy as np
from PySide6.QtCore import QObject, QTimer, Signal

logger = logging.getLogger(__name__)


class CameraFeed(QObject):
    """
    Main-thread camera feed using a QTimer — avoids OpenCV/Qt thread segfault.

    Source:
      int  → local device index  (e.g. 0)
      str  → MJPEG/RTSP URL      (e.g. "http://192.168.1.213:8000/vision/camera/stream")
    """

    frame_ready = Signal(object)   # np.ndarray BGR
    error = Signal(str)

    def __init__(self, fps: int = 20) -> None:
        super().__init__()
        self._source: int | str | None = None
        self._cap: cv2.VideoCapture | None = None
        self._timer = QTimer(self)
        self._timer.setInterval(1000 // fps)
        self._timer.timeout.connect(self._tick)

    # ── public ────────────────────────────────────────────────────────────────

    def set_source(self, source: int | str) -> None:
        """Set camera source and start streaming. Pass an int for local device, str for URL."""
        if isinstance(source, str) and not source.strip():
            logger.warning("CameraFeed.set_source called with empty string — ignoring")
            return
        self._source = source
        self._start()

    def stop(self) -> None:
        self._timer.stop()
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    # ── private ───────────────────────────────────────────────────────────────

    def _start(self) -> None:
        self.stop()
        if self._source is None:
            return
        logger.info("Opening camera source: %s", self._source)
        self._cap = cv2.VideoCapture(self._source)
        if not self._cap.isOpened():
            self._cap.release()
            self._cap = None
            self.error.emit(f"Cannot open camera: {self._source}")
            return
        if isinstance(self._source, int):
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self._timer.start()
        logger.info("Camera feed started: %s", self._source)

    def _tick(self) -> None:
        if self._cap is None:
            self._timer.stop()
            return
        ok, frame = self._cap.read()
        if ok and frame is not None:
            self.frame_ready.emit(frame)
        else:
            self._timer.stop()
            self._cap.release()
            self._cap = None
            self.error.emit("Camera stream lost — reconnect from Setup tab")
