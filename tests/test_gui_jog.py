from __future__ import annotations

import unittest

from vision_gui.robot_arm_gui.jog import build_jog_plan, pose_from_status


CURRENT = {"x": -181.1, "y": 177.4, "z": 80.0, "pitch": 15.8}


class GuiJogTests(unittest.TestCase):
    def test_pose_requires_complete_status(self):
        with self.assertRaisesRegex(ValueError, "pitch"):
            pose_from_status({"x": 1.0, "y": 2.0, "z": 3.0})

    def test_x_plus_sends_full_set_target(self):
        plan = build_jog_plan(CURRENT, "X", 1, table_z=0.0)
        self.assertEqual("SET_TARGET:-179.1:177.4:80.0:15.8", plan.command)

    def test_x_minus_sends_full_set_target(self):
        plan = build_jog_plan(CURRENT, "X", -1, table_z=0.0)
        self.assertEqual("SET_TARGET:-183.1:177.4:80.0:15.8", plan.command)

    def test_z_plus_sends_full_set_target(self):
        plan = build_jog_plan(CURRENT, "Z", 1, table_z=0.0)
        self.assertEqual("SET_TARGET:-181.1:177.4:82.0:15.8", plan.command)

    def test_z_minus_sends_full_set_target(self):
        plan = build_jog_plan(CURRENT, "Z", -1, table_z=0.0)
        self.assertEqual("SET_TARGET:-181.1:177.4:78.0:15.8", plan.command)

    def test_pitch_plus_preserves_other_axes(self):
        plan = build_jog_plan(CURRENT, "PITCH", 1, table_z=0.0)
        self.assertEqual("SET_TARGET:-181.1:177.4:80.0:17.8", plan.command)

    def test_z_minus_refuses_below_table_clearance(self):
        with self.assertRaisesRegex(ValueError, "table clearance"):
            build_jog_plan({"x": 0.0, "y": 170.0, "z": 6.0, "pitch": 0.0}, "Z", -1, table_z=0.0, minimum_clearance=5.0)


if __name__ == "__main__":
    unittest.main()
