from __future__ import annotations

import logging
import sys

from PySide6.QtWidgets import QApplication

from .main_window import MainWindow


logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s %(message)s")


def main() -> None:
    app = QApplication(sys.argv)
    app.setApplicationName("Robot Arm Control")
    app.setStyle("Fusion")

    app.setStyleSheet("""
        QMainWindow, QWidget { background: #1a1a2e; color: #e2e8f0; font-size: 13px; }
        QToolBar { background: #16213e; border-bottom: 1px solid #0f3460; spacing: 4px; padding: 4px; }
        QPushButton {
            background: #0f3460; color: #e2e8f0; border: 1px solid #1a4a7a;
            border-radius: 4px; padding: 6px 14px; min-width: 80px;
        }
        QPushButton:hover { background: #1a4a7a; }
        QPushButton:checked { background: #e94560; border-color: #ff6b8a; }
        QPushButton:disabled { background: #2a2a3e; color: #555; border-color: #333; }
        QPushButton#primaryButton {
            background: #0d7377; border-color: #14a085; font-weight: bold;
        }
        QPushButton#primaryButton:hover { background: #14a085; }
        QPushButton#primaryButton:disabled { background: #2a2a3e; color: #555; }
        QGroupBox {
            border: 1px solid #2a2a4e; border-radius: 6px;
            margin-top: 10px; padding: 8px;
            font-weight: bold; color: #94a3b8;
        }
        QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; }
        QLineEdit, QSpinBox {
            background: #0f0f1a; border: 1px solid #2a2a4e;
            border-radius: 4px; padding: 4px 8px; color: #e2e8f0;
        }
        QLabel { color: #e2e8f0; }
        QCheckBox { color: #e2e8f0; }
        QSlider::groove:horizontal { height: 4px; background: #2a2a4e; border-radius: 2px; }
        QSlider::handle:horizontal {
            width: 14px; height: 14px; margin: -5px 0;
            background: #e94560; border-radius: 7px;
        }
        QStatusBar { background: #16213e; color: #94a3b8; }
    """)

    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
