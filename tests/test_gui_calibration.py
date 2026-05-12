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
from pi_vision_server.calibration import fit_direct_jog_table_z
from vision_gui.robot_arm_gui.realsense_depth import (
    deproject_pixel_to_point_mm,
    fit_realsense_table_z,
    fit_realsense_table_z_two_sample,
    fit_table_plane_from_depth,
    height_above_table_mm,
)


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
    def test_realsense_depth_plane_anchor_fit_calibrates_table_z(self):
        import numpy as np

        depth = np.full((80, 100), 1000, dtype=np.uint16)
        intrinsics = {"fx": 100.0, "fy": 100.0, "ppx": 50.0, "ppy": 40.0}
        markers = [
            {"pixel": {"x": 10, "y": 10}},
            {"pixel": {"x": 90, "y": 10}},
            {"pixel": {"x": 90, "y": 70}},
            {"pixel": {"x": 10, "y": 70}},
        ]
        plane = fit_table_plane_from_depth(depth_image=depth, intrinsics=intrinsics, marker_points=markers, depth_scale=0.001, stride=4)
        claw_point = deproject_pixel_to_point_mm(50, 40, 940.0, intrinsics)
        self.assertAlmostEqual(60.0, height_above_table_mm(claw_point, plane), delta=1.0)
        table_z = fit_realsense_table_z(
            table_plane=plane,
            samples=[
                {"robotZ": 60.0, "heightAboveTableMm": 60.0},
                {"robotZ": 80.0, "heightAboveTableMm": 40.0},
                {"robotZ": 100.0, "heightAboveTableMm": 20.0},
            ],
        )
        self.assertEqual("realsense_depth_plane_anchor_fit", table_z["method"])
        self.assertAlmostEqual(120.0, table_z["z"])
        calibration = empty_calibration()
        calibration["tableZ"] = table_z
        self.assertTrue(table_z_calibrated(calibration))
        self.assertEqual(120.0, estimate_table_z(calibration, 10.0, 20.0))

    def test_two_sample_fit_inverted(self):
        # Inverted convention: physically higher arm = smaller Z number
        # LOW anchor: arm near table, high Z number (e.g. 140), small height (20mm)
        # HIGH anchor: arm far from table, small Z number (e.g. 80), large height (80mm)
        # tableZ estimate = low_z + low_h = 140 + 20 = 160 (or high_z + high_h = 80 + 80 = 160)
        low = {"robotZ": 140.0, "heightAboveTableMm": 20.0}
        high = {"robotZ": 80.0, "heightAboveTableMm": 80.0}
        result = fit_realsense_table_z_two_sample(low_anchor=low, high_anchor=high)
        self.assertEqual("realsense_two_sample", result["method"])
        self.assertTrue(result["zAxisInverted"])
        self.assertAlmostEqual(160.0, result["z"], delta=0.01)
        self.assertAlmostEqual(0.0, result["fit"]["errorMm"], delta=0.01)
        self.assertEqual(80.0, result["hoverRefZ"])
        self.assertNotIn("hoverSlopeZperY", result)  # no Y data → no slope

    def test_two_sample_y_slope_inverted(self):
        # LOW anchor at max Y reach (Y=210), arm near table (Z=140, height=20) → tableZ=160
        # HIGH anchor at moderate Y (Y=150), arm at safe hover (Z=80, height=80)
        # safeHoverZ at Y=210 (nominal) = 160 - 60 = 100
        # slope = (hoverRefZ - nominalHoverAtLowY) / (highY - lowY)
        #       = (80 - 100) / (150 - 210) = -20 / -60 = 1/3
        low = {"robotZ": 140.0, "heightAboveTableMm": 20.0, "robotY": 210.0}
        high = {"robotZ": 80.0, "heightAboveTableMm": 80.0, "robotY": 150.0}
        result = fit_realsense_table_z_two_sample(low_anchor=low, high_anchor=high)
        self.assertTrue(result["zAxisInverted"])
        self.assertAlmostEqual(160.0, result["z"], delta=0.01)
        self.assertEqual(150.0, result["hoverRefY"])
        self.assertEqual(80.0, result["hoverRefZ"])
        self.assertAlmostEqual(1.0 / 3.0, result["hoverSlopeZperY"], delta=0.001)
        # safeHoverZ at Y=210 should be: 80 + (1/3)*(210-150) = 80+20 = 100 ✓ (= tableZ - 60)
        z_at_max_y = result["hoverRefZ"] + result["hoverSlopeZperY"] * (210.0 - result["hoverRefY"])
        self.assertAlmostEqual(100.0, z_at_max_y, delta=0.01)

    def test_two_sample_fit_not_inverted(self):
        # Non-inverted: physically higher arm = larger Z number
        # LOW anchor: near table, small Z (e.g. 20), small height (20mm)
        # HIGH anchor: far from table, large Z (e.g. 80), large height (80mm)
        # tableZ estimate = low_z - low_h = 20 - 20 = 0 (or high_z - high_h = 80 - 80 = 0)
        low = {"robotZ": 20.0, "heightAboveTableMm": 20.0}
        high = {"robotZ": 80.0, "heightAboveTableMm": 80.0}
        result = fit_realsense_table_z_two_sample(low_anchor=low, high_anchor=high)
        self.assertEqual("realsense_two_sample", result["method"])
        self.assertFalse(result["zAxisInverted"])
        self.assertAlmostEqual(0.0, result["z"], delta=0.01)

    def test_two_sample_rejects_insufficient_spread(self):
        low = {"robotZ": 140.0, "heightAboveTableMm": 20.0}
        high = {"robotZ": 138.0, "heightAboveTableMm": 22.0}
        with self.assertRaises(ValueError):
            fit_realsense_table_z_two_sample(low_anchor=low, high_anchor=high)

    def test_multi_sample_outlier_rejection(self):
        # 7 good samples clustered around tableZ=120, plus 2 gross outliers
        good_samples = [
            {"robotZ": 100.0 + i * 5, "heightAboveTableMm": 20.0 - i * 5}
            for i in range(7)
        ]
        bad_samples = [
            {"robotZ": 100.0, "heightAboveTableMm": 0.01},   # near-zero height
            {"robotZ": 100.0, "heightAboveTableMm": 265.0},  # absurdly large height
        ]
        dummy_plane = {"plane": {"a": 0.0, "b": 0.0, "c": 1.0, "d": -1000.0}}
        result = fit_realsense_table_z(
            samples=good_samples + bad_samples,
            table_plane=dummy_plane,
            z_axis_inverted=True,
        )
        self.assertAlmostEqual(120.0, result["z"], delta=2.0)
        self.assertLess(result["fit"]["errorMm"], 5.0)

    def test_direct_jog_calibration_inverted(self):
        result = fit_direct_jog_table_z([
            {"role": "table", "robotY": 200.0, "robotZ": 140.0},
            {"role": "hover", "robotY": 200.0, "robotZ": 80.0},
        ])
        self.assertEqual("direct_jog", result["method"])
        self.assertTrue(result["zAxisInverted"])   # hoverZ 80 < tableZ 140 → inverted
        self.assertAlmostEqual(140.0, result["z"])
        self.assertAlmostEqual(80.0, result["hoverRefZ"])

    def test_direct_jog_calibration_not_inverted(self):
        result = fit_direct_jog_table_z([
            {"role": "table", "robotY": 200.0, "robotZ": 20.0},
            {"role": "hover", "robotY": 200.0, "robotZ": 80.0},
        ])
        self.assertFalse(result["zAxisInverted"])  # hoverZ 80 > tableZ 20 → not inverted
        self.assertAlmostEqual(20.0, result["z"])

    def test_direct_jog_calibration_y_slope(self):
        # Two hover captures at different Y → slope should be computed
        result = fit_direct_jog_table_z([
            {"role": "table", "robotY": 200.0, "robotZ": 140.0},
            {"role": "hover", "robotY": 200.0, "robotZ": 80.0},   # high Y, reference
            {"role": "hover", "robotY": 120.0, "robotZ": 100.0},  # low Y, easier hover
        ])
        # With inverted: at higher Y the elbow drops, requiring lower Z (bigger number = lower for inverted)
        # slope = (Z_at_high_Y - Z_at_low_Y) / (high_Y - low_Y) = (80 - 100) / (200 - 120) = -0.25
        self.assertIn("hoverSlopeZperY", result)
        self.assertAlmostEqual(-0.25, result["hoverSlopeZperY"], delta=0.01)

    def test_direct_jog_calibration_requires_both_roles(self):
        with self.assertRaises(ValueError):
            fit_direct_jog_table_z([{"role": "table", "robotY": 200.0, "robotZ": 140.0}])
        with self.assertRaises(ValueError):
            fit_direct_jog_table_z([{"role": "hover", "robotY": 200.0, "robotZ": 80.0}])

    @unittest.skipUnless(HAS_OPENCV, "OpenCV and NumPy are not installed")
    def test_ransac_plane_fit_ignores_objects_on_table(self):
        import numpy as np

        # Table at 1000 raw units. Objects (boxes) placed in a 20x20 px region
        # at depth 700 (closer to camera = above the table).
        depth = np.full((80, 100), 1000, dtype=np.uint16)
        depth[25:45, 35:55] = 700  # object covers ~6% of the area
        intrinsics = {"fx": 100.0, "fy": 100.0, "ppx": 50.0, "ppy": 40.0}
        markers = [
            {"pixel": {"x": 10, "y": 10}},
            {"pixel": {"x": 90, "y": 10}},
            {"pixel": {"x": 90, "y": 70}},
            {"pixel": {"x": 10, "y": 70}},
        ]
        plane = fit_table_plane_from_depth(depth_image=depth, intrinsics=intrinsics, marker_points=markers, depth_scale=0.001, stride=4)
        # RMS should be low: object points are RANSAC outliers and excluded from the fit
        self.assertLess(plane["rmsErrorMm"], 10.0)
        # Table-surface inlier count should be less than total (object pixels excluded)
        self.assertLess(plane["inlierCount"], plane["totalPointCount"])
        # Height measurement above the fitted table plane should still be accurate
        claw_point = deproject_pixel_to_point_mm(50, 40, 940.0, intrinsics)
        self.assertAlmostEqual(60.0, height_above_table_mm(claw_point, plane), delta=5.0)

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
