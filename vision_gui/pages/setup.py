from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)


class SetupPage(QWidget):
    connect_requested = Signal(str)   # pi_url

    def __init__(self) -> None:
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setSpacing(16)

        group = QGroupBox("Pi Vision Server")
        form = QFormLayout(group)

        self._url = QLineEdit()
        self._url.setPlaceholderText("http://raspberry-pi.local:8000")
        form.addRow("URL:", self._url)

        row = QHBoxLayout()
        self._connect_btn = QPushButton("Connect")
        self._connect_btn.clicked.connect(self._on_connect)
        row.addWidget(self._connect_btn)
        row.addStretch()
        form.addRow("", row)

        self._status = QLabel("Not connected")
        self._status.setObjectName("statusLabel")
        form.addRow("Status:", self._status)

        layout.addWidget(group)

        hint = QGroupBox("What to do here")
        hint_layout = QVBoxLayout(hint)
        hint_layout.addWidget(QLabel(
            "1. Enter the Pi server URL and click Connect.\n"
            "2. Go to Camera — verify the overhead webcam shows the work area.\n"
            "3. Go to Calibrate — place ArUco markers and capture the 4 corners.\n"
            "4. Go to Pick — detect an object and test hover, then full pickup."
        ))
        layout.addWidget(hint)
        layout.addStretch()

    def set_url(self, url: str) -> None:
        self._url.setText(url)

    def get_url(self) -> str:
        return self._url.text().strip()

    def set_status(self, text: str, ok: bool) -> None:
        self._status.setText(text)
        color = "#4ade80" if ok else "#f87171"
        self._status.setStyleSheet(f"color: {color}; font-weight: bold;")

    def _on_connect(self) -> None:
        url = self._url.text().strip()
        if url:
            self.connect_requested.emit(url)
