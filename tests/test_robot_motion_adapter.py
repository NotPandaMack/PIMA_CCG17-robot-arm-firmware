from __future__ import annotations

import unittest

from vision_gui.robot_arm_gui.robot_motion_adapter import RobotMotionAdapter


class FakeEspClient:
    def __init__(self):
        self.sent = []
        self._status = {"x": -181.1, "y": 177.4, "z": 80.0, "pitch": 15.8}

    def status(self):
        return dict(self._status)

    def send_command(self, command):
        self.sent.append(command)
        if command.startswith("SET_TARGET:"):
            _, x, y, z, pitch = command.split(":")
            self._status.update({"x": float(x), "y": float(y), "z": float(z), "pitch": float(pitch)})
        return {"ok": True, "command": command}


class RobotMotionAdapterTests(unittest.TestCase):
    def test_dry_run_logs_web_set_target_without_sending(self):
        client = FakeEspClient()
        logs = []
        adapter = RobotMotionAdapter(client, dry_run=True, logger=logs.append)

        result = adapter.set_target(-181.1, 177.4, 80.0, 15.8)

        self.assertEqual("SET_TARGET:-181.1:177.4:80.0:15.8", result.command)
        self.assertFalse(result.sent)
        self.assertEqual([], client.sent)
        self.assertIn("PyGUI sending same as web UI: SET_TARGET:-181.1:177.4:80.0:15.8", logs)

    def test_jog_axis_uses_latest_pose_and_sends_complete_set_target(self):
        client = FakeEspClient()
        adapter = RobotMotionAdapter(client, dry_run=False)

        result = adapter.jog_axis("X", 2.0, table_z=0.0)

        self.assertEqual("SET_TARGET:-179.1:177.4:80.0:15.8", result.command)
        self.assertEqual(["SET_TARGET:-179.1:177.4:80.0:15.8"], client.sent)

    def test_web_commands_match_website_strings(self):
        client = FakeEspClient()
        adapter = RobotMotionAdapter(client, dry_run=False)

        adapter.open_claw()
        adapter.close_claw_soft()
        adapter.clear_timeline()
        adapter.play_remote_timeline()

        self.assertEqual(["CLAW_OPEN", "CLAW_CLOSE_SOFT", "CLEAR_TIMELINE", "PLAY_REMOTE_TIMELINE"], client.sent)

    def test_adapter_rejects_large_z_lowering(self):
        client = FakeEspClient()
        adapter = RobotMotionAdapter(client, dry_run=False)

        with self.assertRaisesRegex(ValueError, "small jog step"):
            adapter.jog_axis("Z", -10.0, table_z=0.0)


if __name__ == "__main__":
    unittest.main()
