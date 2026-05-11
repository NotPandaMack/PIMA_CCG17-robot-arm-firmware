from __future__ import annotations

from datetime import datetime

from PySide6.QtWidgets import QTextEdit, QVBoxLayout, QWidget


class LogsPanel(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.text = QTextEdit()
        self.text.setReadOnly(True)
        self.text.setObjectName("logsText")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.text)

    def add(self, message: str) -> None:
        stamp = datetime.now().strftime("%H:%M:%S")
        self.text.append(f"[{stamp}] {message}")
