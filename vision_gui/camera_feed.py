from __future__ import annotations

import logging

import cv2
import numpy as np
from PySide6.QtCore import QObject, QTimer, Signal

logger = logging.getLogger(__name__)


class CameraFeed(QObject):
    """
    Main-thread camera feed using a QTimer.

    Source can be:
      - int         → local device index  (e.g. 0)
      - str URL     → MJPEG stream        (e.g. "http://192.168.1.213:8080/video")
    """

    frame_ready = Signal(object)   # np.ndarray BGR
    error = Signal(str)

    def __init__(self, source: int | str = 0, fps: int = 20) -> None:
        super().__init__()
        self._source = source
        self._cap: cv2.VideoCapture | None = None
        self._timer = QTimer(self)
        self._timer.setInterval(1000 // fps)
        self._timer.timeout.connect(self._tick)

    # ── public ────────────────────────────────────────────────────────────────

    def start(self) -> None:
        self.stop()
        self._cap = cv2.VideoCapture(self._source)
        if not self._cap.isOpened():
            self.error.emit(f"Cannot open camera: {self._source}")
            self._cap = None
            return
        if isinstance(self._source, int):
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self._timer.start()
        logger.info("Camera feed started: %s", self._source)

    def stop(self) -> None:
        self._timer.stop()
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    def set_source(self, source: int | str) -> None:
        self._source = source
        self.start()

    # ── private ───────────────────────────────────────────────────────────────

    def _tick(self) -> None:
        if self._cap is None:
            return
        ok, frame = self._cap.read()
        if ok and frame is not None:
            self.frame_ready.emit(frame)
        else:
            self._timer.stop()
            self.error.emit("Camera read failed — stream ended or device lost")
            self._cap = None
