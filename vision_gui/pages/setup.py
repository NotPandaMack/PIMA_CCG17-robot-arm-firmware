from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)


class SetupPage(QWidget):
    connect_requested = Signal(str)   # pi_url
    stream_requested = Signal(str)    # camera stream url or ""

    def __init__(self) -> None:
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setSpacing(16)

        # ── Pi server ────────────────────────────────────────────────────────
        pi_group = QGroupBox("Pi Vision Server")
        pi_form = QFormLayout(pi_group)

        self._url = QLineEdit()
        self._url.setPlaceholderText("http://192.168.1.213:8000")
        pi_form.addRow("URL:", self._url)

        self._connect_btn = QPushButton("Connect")
        self._connect_btn.clicked.connect(lambda: self.connect_requested.emit(self._url.text().strip()))
        pi_form.addRow("", self._connect_btn)

        self._status = QLabel("Not connected")
        self._status.setObjectName("statusLabel")
        pi_form.addRow("Status:", self._status)

        layout.addWidget(pi_group)

        # ── camera source ─────────────────────────────────────────────────────
        cam_group = QGroupBox("Camera Source")
        cam_form = QFormLayout(cam_group)

        cam_form.addRow(QLabel(
            "The overhead camera should be on the Pi and accessible as an MJPEG stream.\n"
            "If the camera is directly on this machine, use the local index instead."
        ))

        self._stream_url = QLineEdit()
        self._stream_url.setPlaceholderText("http://192.168.1.213:8080/video  (Pi MJPEG stream)")
        cam_form.addRow("Stream URL:", self._stream_url)

        stream_btn = QPushButton("Connect Stream")
        stream_btn.clicked.connect(lambda: self.stream_requested.emit(self._stream_url.text().strip()))
        cam_form.addRow("", stream_btn)

        cam_form.addRow(QLabel("— or local camera —"))

        local_row = QHBoxLayout()
        self._local_index = QSpinBox()
        self._local_index.setRange(0, 9)
        self._local_index.setFixedWidth(70)
        local_btn = QPushButton("Use Local Camera")
        local_btn.clicked.connect(lambda: self.stream_requested.emit(""))
        local_row.addWidget(self._local_index)
        local_row.addWidget(local_btn)
        local_row.addStretch()
        cam_form.addRow("Index:", local_row)

        layout.addWidget(cam_group)

        # ── hint ──────────────────────────────────────────────────────────────
        hint_group = QGroupBox("Quick Start")
        hint_layout = QVBoxLayout(hint_group)
        hint_layout.addWidget(QLabel(
            "1. Enter Pi URL → Connect\n"
            "2. Enter Pi camera stream URL → Connect Stream\n"
            "3. Camera tab — verify the overhead view + green detection\n"
            "4. Calibrate tab — capture 4 ArUco corners + pickup pitch\n"
            "5. Pick tab — hover test, then full pickup or auto-pick"
        ))
        layout.addWidget(hint_group)
        layout.addStretch()

    # ── public ────────────────────────────────────────────────────────────────

    def set_url(self, url: str) -> None:
        self._url.setText(url)

    def set_stream_url(self, url: str) -> None:
        self._stream_url.setText(url)

    def set_status(self, text: str, ok: bool) -> None:
        self._status.setText(text)
        self._status.setStyleSheet(f"color: {'#4ade80' if ok else '#f87171'}; font-weight: bold;")
