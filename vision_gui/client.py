from __future__ import annotations

import logging
import threading
from typing import Any

import requests


logger = logging.getLogger(__name__)

_ARUCO_LABELS = ["front-left", "front-right", "back-left", "back-right"]
_ARUCO_ROBOT_XY: dict[str, tuple[float, float]] = {
    "front-left":  (-150.0, 100.0),
    "front-right": ( 150.0, 100.0),
    "back-left":   (-150.0, 280.0),
    "back-right":  ( 150.0, 280.0),
}


class PiClient:
    def __init__(self, base_url: str = "") -> None:
        self.base_url = base_url.rstrip("/")

    def set_url(self, url: str) -> None:
        self.base_url = url.rstrip("/")

    # ── health ──────────────────────────────────────────────────────────────
    def ping(self) -> bool:
        try:
            self._get("/health", timeout=1.0)
            return True
        except Exception:
            return False

    # ── calibration ─────────────────────────────────────────────────────────
    def get_calibration(self) -> dict[str, Any]:
        return self._get("/vision/calibration", timeout=2.0)

    def save_homography(
        self,
        homography: list[list[float]],
        points: list[dict[str, Any]],
    ) -> dict[str, Any]:
        payload: dict[str, Any] = {"homography": homography, "points": points}
        return self._put("/vision/calibration", payload, timeout=2.0)

    def save_pitch(self, pitch_deg: float) -> dict[str, Any]:
        return self._put("/vision/calibration", {"pickupPitchDeg": pitch_deg}, timeout=2.0)

    def save_workspace(self, bounds: dict[str, float]) -> dict[str, Any]:
        return self._put("/vision/config", {"workspace": bounds}, timeout=2.0)

    # ── target + pick ────────────────────────────────────────────────────────
    def post_target(self, payload: dict[str, Any]) -> dict[str, Any]:
        return self._post("/vision/target", payload, timeout=1.0)

    def pick(self, hover_only: bool = False) -> dict[str, Any]:
        suffix = "?hoverOnly=true" if hover_only else ""
        return self._post(f"/vision/pick{suffix}", {}, timeout=3.0)

    def clear_target(self) -> dict[str, Any]:
        return self._post("/vision/clear", {}, timeout=1.0)

    # ── ESP pass-through ─────────────────────────────────────────────────────
    def esp_status(self) -> dict[str, Any]:
        return self._get("/vision/esp/status", timeout=1.0)

    # ── config ───────────────────────────────────────────────────────────────
    def get_config(self) -> dict[str, Any]:
        return self._get("/vision/config", timeout=2.0)

    def patch_config(self, patch: dict[str, Any]) -> dict[str, Any]:
        return self._put("/vision/config", patch, timeout=2.0)

    # ── internals ────────────────────────────────────────────────────────────
    def _get(self, path: str, timeout: float = 1.0) -> dict[str, Any]:
        _assert_bg()
        r = requests.get(f"{self.base_url}{path}", timeout=timeout)
        r.raise_for_status()
        return r.json() if r.content else {}

    def _post(self, path: str, body: dict[str, Any], timeout: float = 1.0) -> dict[str, Any]:
        _assert_bg()
        r = requests.post(f"{self.base_url}{path}", json=body, timeout=timeout)
        r.raise_for_status()
        return r.json() if r.content else {}

    def _put(self, path: str, body: dict[str, Any], timeout: float = 1.0) -> dict[str, Any]:
        _assert_bg()
        r = requests.put(f"{self.base_url}{path}", json=body, timeout=timeout)
        r.raise_for_status()
        return r.json() if r.content else {}


def _assert_bg() -> None:
    if threading.current_thread() is threading.main_thread():
        logger.warning("Network call on main thread — may block UI")
