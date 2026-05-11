from __future__ import annotations

import unittest

from pi_vision_server.validation import ValidationError, validate_website_vision_target_payload


class WebsiteVisionTargetTests(unittest.TestCase):
    def test_accepts_vision_gui_target(self):
        target = validate_website_vision_target_payload(
            {
                "source": "vision_gui",
                "object": "green_object",
                "robotX": -38.1,
                "robotY": 177.4,
                "robotZ": 80.0,
                "pitch": 15.8,
                "confidence": 0.9,
            }
        )

        self.assertEqual(-38.1, target["robotX"])
        self.assertEqual(80.0, target["robotZ"])

    def test_rejects_wrong_source(self):
        with self.assertRaisesRegex(ValidationError, "source"):
            validate_website_vision_target_payload({"source": "other"})

    def test_rejects_missing_pose(self):
        with self.assertRaisesRegex(ValidationError, "robotZ"):
            validate_website_vision_target_payload(
                {
                    "source": "vision_gui",
                    "object": "green_object",
                    "robotX": -38.1,
                    "robotY": 177.4,
                    "pitch": 15.8,
                    "confidence": 0.9,
                }
            )


if __name__ == "__main__":
    unittest.main()
