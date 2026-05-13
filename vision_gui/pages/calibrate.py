from __future__ import annotations

from typing import Any

import numpy as np
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from ..detect import compute_homography, detect_aruco_markers


# Known robot-space XY positions of each ArUco marker ID.
# These are the corners of the calibration sheet you print and place on the table.
# Adjust these if your sheet layout is different.
ARUCO_ROBOT_COORDS: dict[int, tuple[float, float]] = {
    0: (-150.0, 100.0),  # front-left
    1: ( 150.0, 100.0),  # front-right
    2: (-150.0, 280.0),  # back-left
    3: ( 150.0, 280.0),  # back-right
}

POINT_LABELS = {0: "front-left", 1: "front-right", 2: "back-left", 3: "back-right"}


class CalibratePage(QWidget):
    save_homography_requested = Signal(object, object)  # homography, points
    save_pitch_requested = Signal(float)
    status_message = Signal(str)

    def __init__(self) -> None:
        super().__init__()
        layout = QHBoxLayout(self)
        layout.setSpacing(12)

        # ── left: camera view ────────────────────────────────────────────────
        left = QVBoxLayout()
        self._video = QLabel()
        self._video.setFixedSize(640, 480)
        self._video.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._video.setStyleSheet("background: #111; border: 1px solid #333;")
        self._video.setText("No camera — go to Camera tab first")
        left.addWidget(self._video)
        self._marker_label = QLabel("Markers seen: none")
        self._marker_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        left.addWidget(self._marker_label)
        layout.addLayout(left)

        # ── right: controls ──────────────────────────────────────────────────
        right = QVBoxLayout()
        right.setSpacing(16)

        # ── homography ───────────────────────────────────────────────────────
        hom_group = QGroupBox("Step 1 — ArUco Homography (XY mapping)")
        hom_layout = QVBoxLayout(hom_group)
        hom_layout.addWidget(QLabel(
            "Print the ArUco marker sheet (IDs 0-3) and place it on the table.\n"
            "Position the markers at the four corners of the robot workspace.\n"
            "When all 4 are visible in the camera, click Capture & Save."
        ))

        self._captured_points: list[dict[str, Any]] = []
        self._homography: list[list[float]] | None = None

        self._point_labels: dict[int, QLabel] = {}
        for mid, label in POINT_LABELS.items():
            lbl = QLabel(f"ID {mid} ({label}): not seen")
            lbl.setObjectName("pointRow")
            self._point_labels[mid] = lbl
            hom_layout.addWidget(lbl)

        btn_row = QHBoxLayout()
        self._capture_btn = QPushButton("Capture & Save Homography")
        self._capture_btn.setObjectName("primaryButton")
        self._capture_btn.clicked.connect(self._capture_homography)
        btn_row.addWidget(self._capture_btn)
        hom_layout.addLayout(btn_row)

        self._hom_status = QLabel("No homography saved yet.")
        hom_layout.addWidget(self._hom_status)
        right.addWidget(hom_group)

        # ── pitch ────────────────────────────────────────────────────────────
        pitch_group = QGroupBox("Step 2 — Pickup Pitch")
        pitch_layout = QVBoxLayout(pitch_group)
        pitch_layout.addWidget(QLabel(
            "Jog the arm to the table-skim pickup angle using the WebUI manual controls.\n"
            "The pitch shown below comes live from the ESP via the Pi server."
        ))

        self._pitch_readout = QLabel("Pitch: -- °")
        self._pitch_readout.setStyleSheet("font-size: 20px; font-weight: bold;")
        pitch_layout.addWidget(self._pitch_readout)

        self._save_pitch_btn = QPushButton("Save This Pitch")
        self._save_pitch_btn.clicked.connect(self._save_pitch)
        pitch_layout.addWidget(self._save_pitch_btn)

        self._pitch_status = QLabel("No pitch saved yet.")
        pitch_layout.addWidget(self._pitch_status)
        right.addWidget(pitch_group)

        # ── calibration summary ───────────────────────────────────────────────
        sum_group = QGroupBox("Calibration Status")
        sum_layout = QVBoxLayout(sum_group)
        self._cal_summary = QLabel("Not loaded")
        self._cal_summary.setWordWrap(True)
        sum_layout.addWidget(self._cal_summary)
        right.addWidget(sum_group)

        right.addStretch()
        layout.addLayout(right)

        self._current_frame: np.ndarray | None = None
        self._current_markers: list[dict[str, Any]] = []
        self._current_pitch: float | None = None

    # ── public ────────────────────────────────────────────────────────────────

    def update_frame(self, frame: np.ndarray) -> None:
        markers = detect_aruco_markers(frame)
        self._current_frame = frame
        self._current_markers = markers

        annotated = frame.copy()
        import cv2
        seen_ids = {m["id"] for m in markers}
        for m in markers:
            corners = np.array(m["corners"], dtype=np.int32)
            cv2.polylines(annotated, [corners], True, (0, 255, 0), 2)
            cx, cy = int(m["cx"]), int(m["cy"])
            cv2.circle(annotated, (cx, cy), 4, (0, 0, 255), -1)
            cv2.putText(annotated, f"ID {m['id']}", (cx + 6, cy - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)

        h, w, ch = annotated.shape
        img = QImage(annotated.data, w, h, ch * w, QImage.Format.Format_BGR888)
        self._video.setPixmap(QPixmap.fromImage(img).scaled(
            640, 480,
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        ))

        self._marker_label.setText(f"Markers seen: {sorted(seen_ids) or 'none'}")
        for mid, lbl in self._point_labels.items():
            match = next((m for m in markers if m["id"] == mid), None)
            if match:
                lbl.setText(f"ID {mid} ({POINT_LABELS[mid]}): ✓  pixel ({match['cx']:.0f}, {match['cy']:.0f})")
                lbl.setStyleSheet("color: #4ade80;")
            else:
                lbl.setText(f"ID {mid} ({POINT_LABELS[mid]}): not seen")
                lbl.setStyleSheet("color: #f87171;")

    def update_esp_status(self, status: dict[str, Any]) -> None:
        pitch = status.get("pitch")
        if isinstance(pitch, (int, float)):
            self._current_pitch = float(pitch)
            self._pitch_readout.setText(f"Pitch: {pitch:.1f} °")
        else:
            self._current_pitch = None
            self._pitch_readout.setText("Pitch: -- °")

    def update_calibration_summary(self, cal: dict[str, Any]) -> None:
        has_hom = bool(cal.get("homography"))
        pitch = cal.get("pickupPitchDeg")
        table_z = cal.get("tableZ")
        tz_method = (table_z or {}).get("method", "none") if isinstance(table_z, dict) else "none"
        tz_z = (table_z or {}).get("z") if isinstance(table_z, dict) else None

        lines = [
            f"Homography: {'✓ saved' if has_hom else '✗ missing'}",
            f"Pickup pitch: {'✓ ' + str(round(pitch, 1)) + '°' if isinstance(pitch, (int, float)) else '✗ missing'}",
            f"Table Z: {'✓ ' + tz_method + ' z=' + str(round(tz_z, 1)) if isinstance(tz_z, (int, float)) else '✗ — do jog Z calibration in WebUI'}",
        ]
        self._cal_summary.setText("\n".join(lines))

    # ── private ───────────────────────────────────────────────────────────────

    def _capture_homography(self) -> None:
        markers = self._current_markers
        seen = {m["id"]: m for m in markers}
        missing = [mid for mid in ARUCO_ROBOT_COORDS if mid not in seen]
        if missing:
            QMessageBox.warning(self, "Missing Markers",
                                f"Cannot see marker IDs: {missing}\n"
                                "Make sure all 4 ArUco markers are in the camera frame.")
            return

        pixel_pts = []
        robot_pts = []
        point_dicts = []
        for mid, robot_xy in ARUCO_ROBOT_COORDS.items():
            m = seen[mid]
            pixel_pts.append((m["cx"], m["cy"]))
            robot_pts.append(robot_xy)
            point_dicts.append({
                "label": POINT_LABELS[mid],
                "pixel": {"x": m["cx"], "y": m["cy"]},
                "robot": {"x": robot_xy[0], "y": robot_xy[1]},
                "source": "aruco",
            })

        homography = compute_homography(pixel_pts, robot_pts)
        if homography is None:
            QMessageBox.critical(self, "Compute Failed", "Homography computation failed. Try repositioning markers.")
            return

        self._homography = homography
        self._captured_points = point_dicts
        self._hom_status.setText("✓ Homography computed — saving to Pi...")
        self.save_homography_requested.emit(homography, point_dicts)

    def set_homography_saved(self, ok: bool) -> None:
        if ok:
            self._hom_status.setText("✓ Homography saved to Pi server.")
            self._hom_status.setStyleSheet("color: #4ade80;")
        else:
            self._hom_status.setText("✗ Save failed — check Pi connection.")
            self._hom_status.setStyleSheet("color: #f87171;")

    def _save_pitch(self) -> None:
        if self._current_pitch is None:
            QMessageBox.warning(self, "No Pitch", "No arm pitch available — check Pi/ESP connection.")
            return
        self.save_pitch_requested.emit(self._current_pitch)
        self._pitch_status.setText(f"✓ Pitch {self._current_pitch:.1f}° saved to Pi server.")
        self._pitch_status.setStyleSheet("color: #4ade80;")
