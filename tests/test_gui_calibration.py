from __future__ import annotations

import importlib.util
import unittest

from vision_gui.robot_arm_gui.calibration_manager import (
    ChecklistState,
    build_calibration,
    calibration_complete,
    pickup_pose_calibrated,
    table_z_calibrated,
    workspace_bounds_saved,
    empty_calibration,
    fit_side_view_table_z,
    estimate_table_z,
)
from vision_gui.robot_arm_gui.calibration_markers import detect_side_board_markers


HAS_OPENCV = importlib.util.find_spec("cv2") is not None and importlib.util.find_spec("numpy") is not None


class GuiCalibrationTests(unittest.TestCase):
    @unittest.skipUnless(HAS_OPENCV, "OpenCV and NumPy are not installed")
    def test_build_calibration_complete(self):
        calibration = build_calibration(
            camera_width=640,
            camera_height=480,
            origin_pixel=(320, 240),
            points=[
                {"label": "front-left", "pixel": {"x": 100, "y": 400}, "robot": {"x": -150.0, "y": 100.0}},
                {"label": "front-right", "pixel": {"x": 500, "y": 400}, "robot": {"x": 150.0, "y": 100.0}},
                {"label": "back-left", "pixel": {"x": 110, "y": 120}, "robot": {"x": -150.0, "y": 260.0}},
                {"label": "back-right", "pixel": {"x": 490, "y": 120}, "robot": {"x": 150.0, "y": 260.0}},
            ],
            workspace={"xMin": -180.0, "xMax": 180.0, "yMin": 60.0, "yMax": 285.0, "zMin": 0.0, "zMax": 220.0},
            table_z={
                "method": "plane",
                "points": [
                    {"label": "a", "x": 0.0, "y": 100.0, "z": 0.0},
                    {"label": "b", "x": 100.0, "y": 100.0, "z": 0.0},
                    {"label": "c", "x": 0.0, "y": 200.0, "z": 0.0},
                ],
            },
            pickup={
                "pickupPitchDeg": -8.0,
                "skimZ": 10.0,
                "grabOffsetZ": 10.0,
                "hoverZ": 80.0,
                "liftZ": 100.0,
                "clawOpenValue": 0,
                "clawClosedValue": 55,
            },
        )

        self.assertTrue(workspace_bounds_saved(calibration))
        self.assertTrue(table_z_calibrated(calibration))
        self.assertTrue(pickup_pose_calibrated(calibration))
        self.assertTrue(calibration_complete(calibration))

    def test_full_pickup_requires_hover_pass(self):
        base = dict(
            pi_connected=True,
            esp_connected=True,
            webcam_connected=True,
            object_detection_working=True,
            camera_calibration_complete=True,
            workspace_bounds_saved=True,
            table_z_calibrated=True,
            pickup_pose_calibrated=True,
            ghost_preview_passed=True,
            motion_enabled=True,
            target_valid=True,
            estop_inactive=True,
        )
        self.assertFalse(ChecklistState(**base, hover_only_movement_passed=False).full_pickup_ready)
        self.assertTrue(ChecklistState(**base, hover_only_movement_passed=True).full_pickup_ready)

    def test_empty_calibration_does_not_mark_table_z_done(self):
        self.assertFalse(table_z_calibrated(empty_calibration()))

    def test_side_view_visual_fit_calibrates_table_z(self):
        table_z = fit_side_view_table_z(
            table_line={"p1": {"x": 100, "y": 400}, "p2": {"x": 500, "y": 400}},
            samples=[
                {"robotZ": 40.0, "pixel": {"x": 220, "y": 240}, "source": "ESP pose"},
                {"robotZ": 60.0, "pixel": {"x": 220, "y": 280}, "source": "ESP pose"},
                {"robotZ": 80.0, "pixel": {"x": 220, "y": 320}, "source": "website IK draft"},
            ],
            safety_margin_mm=8.0,
        )
        self.assertEqual("side_view_visual_fit", table_z["method"])
        self.assertAlmostEqual(120.0, table_z["z"])
        self.assertAlmostEqual(2.0, table_z["fit"]["pixelsPerRobotMm"])
        calibration = empty_calibration()
        calibration["tableZ"] = table_z
        self.assertTrue(table_z_calibrated(calibration))
        self.assertEqual(120.0, estimate_table_z(calibration, 10.0, 20.0))

    @unittest.skipUnless(HAS_OPENCV, "OpenCV and NumPy are not installed")
    def test_charuco_side_board_detection(self):
        import cv2
        import numpy as np

        aruco = cv2.aruco
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        ids = np.arange(20, 37, dtype=np.int32)
        board = aruco.CharucoBoard((7, 5), 1.0, 0.68, dictionary, ids)
        image = board.generateImage((900, 600), marginSize=24, borderBits=1)
        frame = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        result = detect_side_board_markers(frame)
        self.assertEqual("charuco", result["type"])
        self.assertGreaterEqual(result["charucoCornerCount"], 12)
        self.assertEqual("good", result["quality"])
        self.assertIn(20, result["visibleArucoIds"])


if __name__ == "__main__":
    unittest.main()
