from __future__ import annotations

import logging
import threading
from copy import deepcopy
from datetime import UTC, datetime
from typing import Any

import requests

from .config import DEFAULT_WORKSPACE


logger = logging.getLogger(__name__)


class PiClient:
    def __init__(self, base_url: str, timeout_sec: float = 0.3, mock: bool = False) -> None:
        self.base_url = base_url.rstrip("/")
        self.timeout_sec = timeout_sec
        self.mock = mock
        self._mock_target: dict[str, Any] | None = None
        self._mock_config = {
            "espBaseUrl": "http://fake-esp.local",
            "motionEnabled": False,
            "minConfidence": 0.75,
            "staleTimeoutSec": 2.0,
            "safeHoverZ": 80.0,
            "grabOffsetZ": 10.0,
            "liftZ": 100.0,
            "preApproachRaiseMm": 100.0,
            "zAxisInverted": True,
            "closeClawDegrees": 55,
            "moveDurationMs": 1200,
            "grabWaitAfterMs": 850,
            "workspace": deepcopy(DEFAULT_WORKSPACE),
        }
        self._mock_calibration = {
            "status": "not_calibrated",
            "homography": [],
            "pickupPitchDeg": None,
            "tableZ": {"method": "placeholder", "points": []},
        }

    def set_base_url(self, base_url: str) -> None:
        self.base_url = base_url.rstrip("/")

    def set_mock(self, enabled: bool) -> None:
        self.mock = enabled

    def health(self) -> dict[str, Any]:
        if self.mock:
            return {"ok": True, "mock": True}
        return self._request("GET", "/health", timeout=0.3)

    def get_config(self) -> dict[str, Any]:
        if self.mock:
            return {"ok": True, "config": deepcopy(self._mock_config)}
        return self._request("GET", "/vision/config", timeout=1.0)

    def put_config(self, config_patch: dict[str, Any]) -> dict[str, Any]:
        if self.mock:
            _deep_update(self._mock_config, config_patch)
            return {"ok": True, "config": deepcopy(self._mock_config)}
        return self._request("PUT", "/vision/config", json=config_patch, timeout=1.0)

    def get_target(self) -> dict[str, Any]:
        if self.mock:
            return {"hasTarget": self._mock_target is not None, "target": deepcopy(self._mock_target)}
        return self._request("GET", "/vision/target", timeout=0.3)

    def post_target(self, payload: dict[str, Any]) -> dict[str, Any]:
        if self.mock:
            target = dict(payload)
            target["receivedAt"] = datetime.now(UTC).isoformat().replace("+00:00", "Z")
            self._mock_target = target
            return {"ok": True, "target": deepcopy(target)}
        return self._request("POST", "/vision/target", json=payload, timeout=0.3)

    def clear_target(self) -> dict[str, Any]:
        if self.mock:
            self._mock_target = None
            return {"ok": True, "hasTarget": False}
        return self._request("POST", "/vision/clear", json={}, timeout=0.3)

    def get_calibration(self) -> dict[str, Any]:
        if self.mock:
            return deepcopy(self._mock_calibration)
        return self._request("GET", "/vision/calibration", timeout=1.0)

    def put_calibration(self, calibration: dict[str, Any]) -> dict[str, Any]:
        if self.mock:
            self._mock_calibration = deepcopy(calibration)
            return {"ok": True, "calibration": deepcopy(calibration)}
        return self._request("PUT", "/vision/calibration", json=calibration, timeout=1.0)

    def preview_pick(self, hover_only: bool = False) -> dict[str, Any]:
        if self.mock:
            return self._mock_plan(hover_only)
        suffix = "?hoverOnly=true" if hover_only else ""
        return self._request("POST", f"/vision/pick/preview{suffix}", json={}, timeout=1.0)

    def pick(self, hover_only: bool = False) -> dict[str, Any]:
        if self.mock:
            plan = self._mock_plan(hover_only)
            return {**plan, "sent": plan["ok"] and plan["motionEnabled"]}
        suffix = "?hoverOnly=true" if hover_only else ""
        return self._request("POST", f"/vision/pick{suffix}", json={}, timeout=1.0)

    def get_website_vision_target(self) -> dict[str, Any]:
        if self.mock:
            return {"hasTarget": self._mock_target is not None, "target": deepcopy(self._mock_target)}
        return self._request("GET", "/api/vision-target", timeout=0.5)

    def clear_website_vision_target(self) -> dict[str, Any]:
        if self.mock:
            self._mock_target = None
            return {"ok": True, "hasTarget": False}
        return self._request("POST", "/api/vision-target/clear", json={}, timeout=0.5)

    def esp_status(self) -> dict[str, Any]:
        if self.mock:
            return {"status": "mock", "x": 0.0, "y": 170.0, "z": 80.0, "pitch": -8.0, "estop": False}
        return self._request("GET", "/vision/esp/status", timeout=0.3)

    def _request(self, method: str, path: str, **kwargs: Any) -> dict[str, Any]:
        _warn_if_main_thread(f"{method} {path}")
        if not self.base_url:
            raise RuntimeError("Pi server URL is not configured")
        timeout = kwargs.pop("timeout", self.timeout_sec)
        response = requests.request(method, f"{self.base_url}{path}", timeout=timeout, **kwargs)
        response.raise_for_status()
        if not response.content:
            return {}
        data = response.json()
        if not isinstance(data, dict):
            raise RuntimeError("Pi server returned a non-object JSON response")
        return data

    def _mock_plan(self, hover_only: bool) -> dict[str, Any]:
        target = self._mock_target
        errors = []
        if target is None:
            errors.append("no target is stored")
        elif target.get("confidence", 0.0) < self._mock_config["minConfidence"]:
            errors.append("target confidence is below minimum")
        elif target.get("robotX") is None or target.get("robotY") is None:
            errors.append("target must include robotX and robotY")

        x = float(target.get("robotX", 35.0)) if target else 35.0
        y = float(target.get("robotY", 210.0)) if target else 210.0
        table_z = float((self._mock_calibration.get("tableZ") or {}).get("z", 120.0))
        z_sign = -1.0 if self._mock_config.get("zAxisInverted", True) else 1.0
        hover_z = table_z + (z_sign * float(self._mock_config["safeHoverZ"]))
        grab_z = table_z + (z_sign * float(self._mock_config["grabOffsetZ"]))
        lift_z = table_z + (z_sign * float(self._mock_config["liftZ"]))
        safe_raise_z = table_z + (z_sign * float(self._mock_config.get("preApproachRaiseMm", 100.0)))
        pitch = float(self._mock_calibration.get("pickupPitchDeg") or -8.0)
        commands = [
            "STOP",
            "CLEAR_TIMELINE",
            "CLAW_OPEN",
            f"ADD_KEYFRAME:MOVE:{x:.1f}:{y:.1f}:{safe_raise_z:.1f}:{pitch:.1f}:0:-1:1200:200",
            f"ADD_KEYFRAME:MOVE:{x:.1f}:{y:.1f}:{hover_z:.1f}:{pitch:.1f}:0:-1:1200:200",
        ]
        if not hover_only:
            commands.extend(
                [
                    f"ADD_KEYFRAME:MOVE:{x:.1f}:{y:.1f}:{grab_z:.1f}:{pitch:.1f}:0:-1:1200:200",
                    f"ADD_KEYFRAME:GRAB:{x:.1f}:{y:.1f}:{grab_z:.1f}:{pitch:.1f}:0:55:1200:850",
                    f"ADD_KEYFRAME:MOVE:{x:.1f}:{y:.1f}:{lift_z:.1f}:{pitch:.1f}:0:55:1200:200",
                ]
            )
        commands.append("PLAY_REMOTE_TIMELINE")
        return {
            "ok": not errors,
            "errors": errors,
            "hoverOnly": hover_only,
            "motionEnabled": bool(self._mock_config["motionEnabled"]),
            "willSendMotion": not errors and bool(self._mock_config["motionEnabled"]),
            "target": deepcopy(target),
            "calculated": {
                "robotX": x,
                "robotY": y,
                "tableZ": table_z,
                "safeRaiseZ": safe_raise_z,
                "hoverZ": hover_z,
                "skimGrabZ": grab_z,
                "liftZ": lift_z,
                "zAxisInverted": bool(self._mock_config.get("zAxisInverted", True)),
                "pickupPitchDeg": pitch,
                "targetAgeSec": 0.1,
            },
            "workspace": {"ok": True, "errors": [], "bounds": deepcopy(self._mock_config["workspace"])},
            "calibration": {
                "isCalibrated": bool(self._mock_calibration.get("homography")),
                "hasHomography": bool(self._mock_calibration.get("homography")),
                "hasPickupPitch": self._mock_calibration.get("pickupPitchDeg") is not None,
                "tableZStatus": "placeholder",
            },
            "commands": commands,
        }


def _deep_update(target: dict[str, Any], source: dict[str, Any]) -> None:
    for key, value in source.items():
        if isinstance(value, dict) and isinstance(target.get(key), dict):
            _deep_update(target[key], value)
        else:
            target[key] = value


def _warn_if_main_thread(operation: str) -> None:
    if threading.current_thread() is threading.main_thread():
        logger.warning("Network call made from main thread: %s", operation)
