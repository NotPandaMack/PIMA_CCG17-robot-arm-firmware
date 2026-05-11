from __future__ import annotations

from copy import deepcopy
from datetime import UTC, datetime
from typing import Any

import requests


class WebsiteClient:
    def __init__(self, base_url: str, timeout_sec: float = 1.0, mock: bool = False) -> None:
        self.base_url = base_url.rstrip("/")
        self.timeout_sec = timeout_sec
        self.mock = mock
        self._mock_target: dict[str, Any] | None = None

    def post_vision_target(self, payload: dict[str, Any]) -> dict[str, Any]:
        if self.mock:
            target = dict(payload)
            target["receivedAt"] = datetime.now(UTC).isoformat().replace("+00:00", "Z")
            self._mock_target = target
            return {"ok": True, "target": deepcopy(target)}
        response = requests.post(f"{self.base_url}/api/vision-target", json=payload, timeout=self.timeout_sec)
        response.raise_for_status()
        data = response.json()
        if not isinstance(data, dict):
            raise RuntimeError("website returned non-object JSON")
        return data

    def get_vision_target(self) -> dict[str, Any]:
        if self.mock:
            return {"hasTarget": self._mock_target is not None, "target": deepcopy(self._mock_target)}
        response = requests.get(f"{self.base_url}/api/vision-target", timeout=0.3)
        response.raise_for_status()
        data = response.json()
        if not isinstance(data, dict):
            raise RuntimeError("website returned non-object JSON")
        return data

    def get_manual_control_state(self) -> dict[str, Any]:
        if self.mock:
            return {"hasState": False, "state": None}
        response = requests.get(f"{self.base_url}/api/manual-control-state", timeout=0.3)
        response.raise_for_status()
        data = response.json()
        if not isinstance(data, dict):
            raise RuntimeError("website returned non-object JSON")
        return data
