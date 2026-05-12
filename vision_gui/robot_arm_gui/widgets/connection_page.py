from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QFormLayout,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from .status_widgets import ChecklistPanel, StatusPill


class ConnectionPage(QWidget):
    save_requested = Signal()
    test_pi_requested = Signal()
    test_esp_requested = Signal()
    test_camera_requested = Signal()
    test_side_camera_requested = Signal()
    auto_detect_requested = Signal()
    start_setup_requested = Signal()

    def __init__(self, checklist: ChecklistPanel) -> None:
        super().__init__()
        self.checklist = checklist
        layout = QVBoxLayout(self)
        layout.setSpacing(14)

        hero = QFrame()
        hero.setObjectName("heroPanel")
        hero_layout = QVBoxLayout(hero)
        title = QLabel("Welcome to Robot Arm Setup")
        title.setObjectName("heroTitle")
        subtitle = QLabel("Main PC webcam -> Raspberry Pi safety server -> ESP robot arm")
        subtitle.setObjectName("mutedText")
        self.start_button = QPushButton("Start Setup")
        self.start_button.setObjectName("primaryButton")
        self.start_button.clicked.connect(self.start_setup_requested.emit)
        hero_layout.addWidget(title)
        hero_layout.addWidget(subtitle)
        hero_layout.addWidget(self.start_button)
        layout.addWidget(hero)

        grid = QGridLayout()
        layout.addLayout(grid, 1)

        settings_card = QFrame()
        settings_card.setObjectName("panel")
        form = QFormLayout(settings_card)
        self.pi_url = QLineEdit()
        self.website_url = QLineEdit()
        self.esp_url = QLineEdit()
        self.camera_index = QSpinBox()
        self.camera_index.setRange(0, 12)
        self.side_camera_url = QLineEdit()
        self.side_camera_url.setPlaceholderText("rtmps://desktop-ip:1936/live")
        self.side_camera_url.setReadOnly(True)
        self.side_camera_stream_key = QLineEdit()
        self.side_camera_stream_key.setReadOnly(True)
        self.mock_pi = QCheckBox("Use mock Pi mode")
        self.fake_esp = QCheckBox("Use fake ESP status")
        form.addRow("Raspberry Pi URL", self.pi_url)
        form.addRow("Website/Pi UI URL", self.website_url)
        form.addRow("ESP URL", self.esp_url)
        form.addRow("Webcam index", self.camera_index)
        form.addRow("Camo RTMPS server URL", self.side_camera_url)
        form.addRow("Camo stream key", self.side_camera_stream_key)
        form.addRow("", self.mock_pi)
        form.addRow("", self.fake_esp)

        button_row = QHBoxLayout()
        self.auto_detect_button = QPushButton("Auto-detect Pi")
        self.test_pi_button = QPushButton("Test Pi Connection")
        self.test_esp_button = QPushButton("Test ESP Connection")
        self.test_camera_button = QPushButton("Test Webcam")
        self.test_side_camera_button = QPushButton("Test RTMPS Relay")
        self.save_button = QPushButton("Save Settings")
        self.save_button.setObjectName("primaryButton")
        for button in (
            self.auto_detect_button,
            self.test_pi_button,
            self.test_esp_button,
            self.test_camera_button,
            self.test_side_camera_button,
            self.save_button,
        ):
            button_row.addWidget(button)
        form.addRow(button_row)
        self.auto_detect_button.clicked.connect(self.auto_detect_requested.emit)
        self.test_pi_button.clicked.connect(self.test_pi_requested.emit)
        self.test_esp_button.clicked.connect(self.test_esp_requested.emit)
        self.test_camera_button.clicked.connect(self.test_camera_requested.emit)
        self.test_side_camera_button.clicked.connect(self.test_side_camera_requested.emit)
        self.save_button.clicked.connect(self.save_requested.emit)

        status_card = QFrame()
        status_card.setObjectName("panel")
        status_layout = QVBoxLayout(status_card)
        status_layout.addWidget(QLabel("Connection Status"))
        self.pi_status = StatusPill("Pi disconnected", "red")
        self.esp_status = StatusPill("ESP disconnected", "red")
        self.camera_status = StatusPill("Camera disconnected", "red")
        self.side_camera_status = StatusPill("Side camera disconnected", "red")
        status_layout.addWidget(self.pi_status)
        status_layout.addWidget(self.esp_status)
        status_layout.addWidget(self.camera_status)
        status_layout.addWidget(self.side_camera_status)
        status_layout.addStretch(1)

        grid.addWidget(settings_card, 0, 0)
        grid.addWidget(status_card, 0, 1)
        grid.addWidget(self.checklist, 0, 2)
        grid.setColumnStretch(0, 2)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(2, 1)

    def set_from_settings(self, settings: dict) -> None:
        self.pi_url.setText(settings.get("piUrl", ""))
        self.website_url.setText(settings.get("websiteUrl", settings.get("piUrl", "")))
        self.esp_url.setText(settings.get("espUrl", ""))
        self.camera_index.setValue(int(settings.get("cameraIndex", 0)))
        self.side_camera_url.setText(settings.get("sideCameraUrl", ""))
        self.side_camera_stream_key.setText(settings.get("sideCameraStreamKey", "side"))
        self.mock_pi.setChecked(bool(settings.get("mockPi", False)))
        self.fake_esp.setChecked(bool(settings.get("fakeEsp", False)))

    def to_settings_patch(self) -> dict:
        return {
            "piUrl": self.pi_url.text().strip(),
            "websiteUrl": self.website_url.text().strip(),
            "espUrl": self.esp_url.text().strip(),
            "cameraIndex": self.camera_index.value(),
            "sideCameraUrl": self.side_camera_url.text().strip(),
            "sideCameraStreamKey": self.side_camera_stream_key.text().strip(),
            "mockPi": self.mock_pi.isChecked(),
            "fakeEsp": self.fake_esp.isChecked(),
        }

    def update_statuses(self, pi: bool, esp: bool, camera: bool) -> None:
        self.pi_status.set_state("Pi connected" if pi else "Pi disconnected", "green" if pi else "red")
        self.esp_status.set_state("ESP connected" if esp else "ESP disconnected", "green" if esp else "red")
        self.camera_status.set_state("Camera connected" if camera else "Camera disconnected", "green" if camera else "red")

    def update_side_camera_status(self, connected: bool | None, message: str | None = None) -> None:
        if connected is None:
            self.side_camera_status.set_state(message or "Side camera not tested", "yellow")
            return
        default = "Side camera connected" if connected else "Side camera disconnected"
        self.side_camera_status.set_state(message or default, "green" if connected else "red")

    def set_side_camera_rtmps_details(self, server_url: str, stream_key: str) -> None:
        self.side_camera_url.setText(server_url)
        self.side_camera_stream_key.setText(stream_key)
