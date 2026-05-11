from __future__ import annotations

import unittest

from vision_gui.robot_arm_gui.geometry import map_widget_point_to_frame


class GuiClickMappingTests(unittest.TestCase):
    def test_maps_center_without_letterbox(self):
        result = map_widget_point_to_frame(
            widget_width=1280,
            widget_height=720,
            frame_width=640,
            frame_height=360,
            widget_x=640,
            widget_y=360,
        )

        self.assertEqual(320, result["pixelX"])
        self.assertEqual(180, result["pixelY"])

    def test_maps_with_vertical_letterbox_padding(self):
        result = map_widget_point_to_frame(
            widget_width=1000,
            widget_height=800,
            frame_width=640,
            frame_height=360,
            widget_x=500,
            widget_y=400,
        )

        self.assertEqual(320, result["pixelX"])
        self.assertEqual(180, result["pixelY"])
        self.assertAlmostEqual(118.75, result["displayRect"][1], places=2)

    def test_rejects_click_in_black_bar(self):
        result = map_widget_point_to_frame(
            widget_width=1000,
            widget_height=800,
            frame_width=640,
            frame_height=360,
            widget_x=500,
            widget_y=20,
        )

        self.assertIsNone(result)

    def test_maps_with_horizontal_letterbox_padding(self):
        result = map_widget_point_to_frame(
            widget_width=1200,
            widget_height=900,
            frame_width=640,
            frame_height=720,
            widget_x=600,
            widget_y=450,
        )

        self.assertEqual(320, result["pixelX"])
        self.assertEqual(360, result["pixelY"])
        self.assertAlmostEqual(200.0, result["displayRect"][0], places=2)


if __name__ == "__main__":
    unittest.main()
