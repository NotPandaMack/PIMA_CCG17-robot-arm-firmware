from __future__ import annotations

import unittest
from datetime import UTC, datetime, timedelta

from pi_vision_server.calibration import get_table_z, pixel_to_robot_homography
from pi_vision_server.config import VisionConfig, WorkspaceBounds
from pi_vision_server.planner import build_pick_plan, execute_plan
from pi_vision_server.validation import ValidationError, validate_target_payload


def fresh_target(**overrides):
    target = {
        "type": "object_detected",
        "object": "green_object",
        "pixelX": 420,
        "pixelY": 310,
        "robotX": 35.0,
        "robotY": 210.0,
        "confidence": 0.92,
        "receivedAt": datetime.now(UTC).isoformat().replace("+00:00", "Z"),
    }
    target.update(overrides)
    return target


def calibrated(**overrides):
    calibration = {
        "status": "calibrated",
        "pickupPitchDeg": -8.0,
        "homography": [[1.0, 0.0, -100.0], [0.0, 1.0, 50.0], [0.0, 0.0, 1.0]],
        "tableZ": {
            "method": "plane",
            "points": [
                {"label": "a", "x": 0.0, "y": 100.0, "z": 120.0},
                {"label": "b", "x": 100.0, "y": 100.0, "z": 120.0},
                {"label": "c", "x": 0.0, "y": 200.0, "z": 120.0},
            ],
        },
    }
    calibration.update(overrides)
    return calibration


class FakeEspClient:
    def __init__(self, estop=False):
        self.sent = []
        self.estop = estop

    def get_status(self):
        return {"estop": self.estop}

    def send_command(self, command):
        self.sent.append(command)


class VisionSafetyTests(unittest.TestCase):
    def test_rejects_invalid_target_payload(self):
        with self.assertRaises(ValidationError):
            validate_target_payload({"type": "object_detected", "object": "", "pixelX": 1, "pixelY": 2, "confidence": 0.5})

    def test_motion_disabled_returns_commands_but_sends_nothing(self):
        config = VisionConfig(motion_enabled=False)
        plan = build_pick_plan(fresh_target(), config, calibrated())
        self.assertTrue(plan["ok"])
        self.assertFalse(plan["willSendMotion"])
        self.assertIn("PLAY_REMOTE_TIMELINE", plan["commands"])

        esp = FakeEspClient()
        result = execute_plan(plan, esp)
        self.assertFalse(result["sent"])
        self.assertEqual([], esp.sent)

    def test_hover_only_removes_lower_grab_and_lift_steps(self):
        plan = build_pick_plan(fresh_target(), VisionConfig(), calibrated(), hover_only=True)
        joined = "\n".join(plan["commands"])
        self.assertTrue(plan["ok"])
        # Pre-approach raise step must be present (safe_raise_z = table_z - pre_approach_raise_mm = 120 - 100 = 20)
        self.assertIn("ADD_KEYFRAME:MOVE:35.0:210.0:20.0:-8.0", joined)
        # Hover approach step must be present
        self.assertIn("ADD_KEYFRAME:MOVE:35.0:210.0:40.0:-8.0", joined)
        # Grab and lower-to-grab steps must NOT be present in hover-only mode
        self.assertNotIn("ADD_KEYFRAME:GRAB", joined)
        self.assertNotIn(":110.0:-8.0", joined)

    def test_stale_target_is_blocked(self):
        old = datetime.now(UTC) - timedelta(seconds=10)
        plan = build_pick_plan(fresh_target(receivedAt=old.isoformat().replace("+00:00", "Z")), VisionConfig(), calibrated())
        self.assertFalse(plan["ok"])
        self.assertIn("stale", " ".join(plan["errors"]))

    def test_low_confidence_is_blocked(self):
        plan = build_pick_plan(fresh_target(confidence=0.1), VisionConfig(), calibrated())
        self.assertFalse(plan["ok"])
        self.assertIn("confidence", " ".join(plan["errors"]))

    def test_missing_robot_coordinates_are_blocked(self):
        target = fresh_target()
        target.pop("robotX")
        plan = build_pick_plan(target, VisionConfig(), calibrated())
        self.assertFalse(plan["ok"])
        self.assertIn("robotX", " ".join(plan["errors"]))

    def test_workspace_bounds_are_blocked(self):
        config = VisionConfig(workspace=WorkspaceBounds(x_min=-10.0, x_max=10.0, y_min=60.0, y_max=285.0))
        plan = build_pick_plan(fresh_target(robotX=35.0), config, calibrated())
        self.assertFalse(plan["ok"])
        self.assertIn("outside", " ".join(plan["errors"]))

    def test_uncalibrated_pickup_is_blocked(self):
        plan = build_pick_plan(fresh_target(), VisionConfig(), {"status": "not_calibrated", "homography": [], "tableZ": {"points": []}})
        self.assertFalse(plan["ok"])
        self.assertIn("tableZ", " ".join(plan["errors"]))

    def test_missing_table_z_never_defaults_to_zero(self):
        plan = build_pick_plan(fresh_target(), VisionConfig(), calibrated(tableZ={"method": "placeholder", "points": []}))
        self.assertFalse(plan["ok"])
        self.assertIn("tableZ is missing", " ".join(plan["errors"]))

    def test_estop_blocks_execution_when_motion_enabled(self):
        config = VisionConfig(motion_enabled=True)
        plan = build_pick_plan(fresh_target(), config, calibrated())
        esp = FakeEspClient(estop=True)
        result = execute_plan(plan, esp)
        self.assertFalse(result["sent"])
        self.assertEqual([], esp.sent)
        self.assertIn("ESTOP", " ".join(result["errors"]))

    def test_homography_conversion(self):
        result = pixel_to_robot_homography(calibrated(), 120.0, 200.0)
        self.assertEqual((20.0, 250.0), result)

    def test_side_view_table_z_is_used(self):
        calibration = calibrated(tableZ={"method": "side_view_visual_fit", "z": 120.0, "samples": []})
        self.assertEqual(120.0, get_table_z(calibration, 35.0, 210.0))
        plan = build_pick_plan(fresh_target(), VisionConfig(), calibration, hover_only=True)
        self.assertEqual(120.0, plan["calculated"]["tableZ"])
        self.assertEqual(40.0, plan["calculated"]["hoverZ"])

    def test_realsense_table_z_is_used(self):
        calibration = calibrated(tableZ={"method": "realsense_depth_plane_anchor_fit", "z": 120.0, "samples": []})
        self.assertEqual(120.0, get_table_z(calibration, 35.0, 210.0))
        plan = build_pick_plan(fresh_target(), VisionConfig(), calibration, hover_only=True)
        self.assertTrue(plan["ok"])
        self.assertEqual(40.0, plan["calculated"]["hoverZ"])

    def test_two_sample_table_z_is_used(self):
        calibration = calibrated(tableZ={"method": "realsense_two_sample", "z": 120.0})
        self.assertEqual(120.0, get_table_z(calibration, 35.0, 210.0))
        plan = build_pick_plan(fresh_target(), VisionConfig(), calibration, hover_only=True)
        self.assertTrue(plan["ok"])
        self.assertEqual(40.0, plan["calculated"]["hoverZ"])


if __name__ == "__main__":
    unittest.main()
