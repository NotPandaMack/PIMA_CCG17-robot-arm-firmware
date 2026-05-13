from __future__ import annotations

import logging

import cv2
import numpy as np
from PySide6.QtCore import QThread, Signal

logger = logging.getLogger(__name__)


class CameraThread(QThread):
    frame_ready = Signal(object)   # emits np.ndarray BGR frame
    error = Signal(str)

    def __init__(self, source: int | str = 0) -> None:
        super().__init__()
        self.source = source
        self._running = False

    def run(self) -> None:
        self._running = True
        cap = cv2.VideoCapture(self.source)
        if not cap.isOpened():
            self.error.emit(f"Cannot open camera: {self.source}")
            return
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        logger.info("Camera thread started: %s", self.source)
        while self._running:
            ok, frame = cap.read()
            if not ok:
                self.error.emit("Camera read failed")
                break
            self.frame_ready.emit(frame)
            self.msleep(33)  # ~30 fps
        cap.release()
        logger.info("Camera thread stopped")

    def stop(self) -> None:
        self._running = False
        self.wait(2000)
