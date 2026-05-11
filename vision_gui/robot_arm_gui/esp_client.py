from __future__ import annotations

from typing import Any
from urllib.parse import quote

import requests


class EspClient:
    def __init__(self, base_url: str, timeout_sec: float = 1.8, fake: bool = False) -> None:
        self.base_url = base_url.rstrip("/")
        self.timeout_sec = timeout_sec
        self.fake = fake
        self.fake_status: dict[str, Any] = {
            "status": "fake ready",
            "toolMode": "TIP",
            "x": 0.0,
            "y": 170.0,
            "z": 80.0,
            "pitch": -8.0,
            "clawTicks": 0,
            "speed": 1,
            "keyframeCount": 0,
            "timelinePlaying": False,
            "estop": False,
            "lastCommand": "none",
        }

    def set_base_url(self, base_url: str) -> None:
        self.base_url = base_url.rstrip("/")

    def set_fake(self, enabled: bool) -> None:
        self.fake = enabled

    def status(self) -> dict[str, Any]:
        if self.fake:
            return dict(self.fake_status)
        if not self.base_url:
            raise RuntimeError("ESP URL is not configured")
        response = requests.get(f"{self.base_url}/status", timeout=self.timeout_sec)
        response.raise_for_status()
        data = response.json()
        if not isinstance(data, dict):
            raise RuntimeError("ESP returned a non-object JSON response")
        return data

    def send_command(self, command: str) -> dict[str, Any]:
        if self.fake:
            self.fake_status["lastCommand"] = command
            self.fake_status["commandCount"] = int(self.fake_status.get("commandCount", 0)) + 1
            if command == "ESTOP":
                self.fake_status["estop"] = True
            elif command == "CLEAR_ESTOP":
                self.fake_status["estop"] = False
            elif command.startswith("SET_TARGET:"):
                parts = command.split(":")
                if len(parts) >= 5:
                    self.fake_status.update(
                        {
                            "x": float(parts[1]),
                            "y": float(parts[2]),
                            "z": float(parts[3]),
                            "pitch": float(parts[4]),
                        }
                    )
            return {"ok": True, "command": command, "fake": True}
        if not self.base_url:
            raise RuntimeError("ESP URL is not configured")
        response = requests.get(f"{self.base_url}/command?cmd={quote(command)}", timeout=self.timeout_sec)
        response.raise_for_status()
        data = response.json()
        return data if isinstance(data, dict) else {"ok": True, "command": command}


def ik_move_command(dx: float, dy: float, dz: float, dp: float) -> str:
    return f"IKMOVE:{dx:.2f}:{dy:.2f}:{dz:.2f}:{dp:.2f}"
