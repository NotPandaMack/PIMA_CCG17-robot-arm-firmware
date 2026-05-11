from __future__ import annotations

import ast
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
MAIN_WINDOW = REPO_ROOT / "vision_gui" / "robot_arm_gui" / "main_window.py"
PI_CLIENT = REPO_ROOT / "vision_gui" / "robot_arm_gui" / "pi_client.py"
ESP_CLIENT = REPO_ROOT / "vision_gui" / "robot_arm_gui" / "esp_client.py"


class GuiResponsivenessTests(unittest.TestCase):
    def test_main_window_has_no_top_level_opencv_import(self):
        tree = ast.parse(MAIN_WINDOW.read_text(encoding="utf-8"))
        imports = [node for node in tree.body if isinstance(node, (ast.Import, ast.ImportFrom))]
        imported_names = []
        for node in imports:
            if isinstance(node, ast.Import):
                imported_names.extend(alias.name for alias in node.names)
            else:
                imported_names.append(node.module or "")
        self.assertNotIn("cv2", imported_names)

    def test_main_window_constructor_does_not_refresh_or_probe_hardware(self):
        tree = ast.parse(MAIN_WINDOW.read_text(encoding="utf-8"))
        main_window = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "MainWindow")
        init = next(node for node in main_window.body if isinstance(node, ast.FunctionDef) and node.name == "__init__")
        called = {node.func.attr for node in ast.walk(init) if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)}
        self.assertNotIn("refresh_status", called)
        self.assertNotIn("test_pi_connection", called)
        self.assertNotIn("test_esp_connection", called)
        self.assertNotIn("test_webcam", called)

    def test_clients_default_to_short_status_timeout(self):
        self.assertIn("timeout_sec: float = 0.3", PI_CLIENT.read_text(encoding="utf-8"))
        self.assertIn("timeout_sec: float = 0.3", ESP_CLIENT.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
