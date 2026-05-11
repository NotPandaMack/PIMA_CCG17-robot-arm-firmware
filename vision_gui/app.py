from __future__ import annotations

import sys
import logging

from PySide6.QtWidgets import QApplication

from robot_arm_gui.main_window import MainWindow


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    app = QApplication(sys.argv)
    app.setApplicationName("Robot Arm Control Center")
    app.setOrganizationName("PIMA Robot Arm")

    window = MainWindow()
    window.resize(1440, 920)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
