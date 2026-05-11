from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from pi_vision_server.config import config_to_dict, load_config, update_config


class VisionConfigTests(unittest.TestCase):
    def test_update_config_saves_motion_and_workspace(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "vision_server_config.json"
            config = update_config(
                {
                    "espBaseUrl": "http://192.168.1.60/",
                    "motionEnabled": True,
                    "workspace": {"xMin": -150.0, "xMax": 150.0},
                },
                path,
            )

            self.assertTrue(config.motion_enabled)
            self.assertEqual("http://192.168.1.60", config.esp_base_url)
            self.assertEqual(-150.0, config.workspace.x_min)
            self.assertEqual(150.0, load_config(path).workspace.x_max)

    def test_update_config_rejects_unknown_fields(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "vision_server_config.json"
            with self.assertRaisesRegex(ValueError, "unsupported"):
                update_config({"dangerMode": True}, path)

    def test_update_config_rejects_invalid_workspace(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "vision_server_config.json"
            with self.assertRaisesRegex(ValueError, "xMin"):
                update_config({"workspace": {"xMin": 10.0, "xMax": -10.0}}, path)

    def test_config_to_dict_uses_public_keys(self):
        data = config_to_dict(load_config(Path("/tmp/missing-robot-arm-config.json")))
        self.assertIn("motionEnabled", data)
        self.assertIn("workspace", data)
        self.assertIn("xMin", data["workspace"])


if __name__ == "__main__":
    unittest.main()
