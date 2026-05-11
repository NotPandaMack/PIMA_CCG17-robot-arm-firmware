from __future__ import annotations

import importlib.util
import unittest

HAS_OPENCV = importlib.util.find_spec("cv2") is not None and importlib.util.find_spec("numpy") is not None

if HAS_OPENCV:
    import cv2
    import numpy as np

    from vision_gui.robot_arm_gui.detection_worker import HSVProfile, detect_green_object



@unittest.skipUnless(HAS_OPENCV, "OpenCV and NumPy are not installed")
class GuiDetectionTests(unittest.TestCase):
    def test_detects_synthetic_green_object(self):
        frame = np.zeros((240, 320, 3), dtype=np.uint8)
        cv2.circle(frame, (150, 90), 24, (0, 255, 0), -1)

        result = detect_green_object(frame, HSVProfile(min_area=100.0))

        self.assertTrue(result.found)
        self.assertAlmostEqual(150, result.pixel_x, delta=2)
        self.assertAlmostEqual(90, result.pixel_y, delta=2)
        self.assertGreater(result.confidence, 0.7)

    def test_rejects_small_noise(self):
        frame = np.zeros((240, 320, 3), dtype=np.uint8)
        cv2.circle(frame, (20, 20), 2, (0, 255, 0), -1)

        result = detect_green_object(frame, HSVProfile(min_area=100.0))

        self.assertFalse(result.found)


if __name__ == "__main__":
    unittest.main()
