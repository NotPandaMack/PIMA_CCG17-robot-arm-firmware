from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG_PATH = REPO_ROOT / "config" / "vision_server_config.json"


@dataclass(frozen=True)
class WorkspaceBounds:
    x_min: float = -180.0
    x_max: float = 180.0
    y_min: float = 60.0
    y_max: float = 285.0
    z_min: float = 0.0
    z_max: float = 220.0


@dataclass(frozen=True)
class VisionConfig:
    esp_base_url: str = "http://ESP8266_IP"
    motion_enabled: bool = False
    min_confidence: float = 0.75
    stale_timeout_sec: float = 2.0
    safe_hover_z: float = 80.0
    grab_offset_z: float = 10.0
    lift_z: float = 100.0
    close_claw_degrees: int = 55
    move_duration_ms: int = 1200
    grab_wait_after_ms: int = 850
    workspace: WorkspaceBounds = WorkspaceBounds()


def _as_float(value: Any, default: float) -> float:
    if isinstance(value, (int, float)):
        return float(value)
    return default


def _as_int(value: Any, default: int) -> int:
    if isinstance(value, int):
        return value
    if isinstance(value, float):
        return int(value)
    return default


def _as_bool(value: Any, default: bool) -> bool:
    if isinstance(value, bool):
        return value
    return default


def load_config(path: Path = DEFAULT_CONFIG_PATH) -> VisionConfig:
    if not path.exists():
        return VisionConfig()

    with path.open("r", encoding="utf-8") as file:
        raw = json.load(file)

    defaults = VisionConfig()
    bounds_defaults = WorkspaceBounds()
    workspace_raw = raw.get("workspace", {})
    workspace = WorkspaceBounds(
        x_min=_as_float(workspace_raw.get("xMin"), bounds_defaults.x_min),
        x_max=_as_float(workspace_raw.get("xMax"), bounds_defaults.x_max),
        y_min=_as_float(workspace_raw.get("yMin"), bounds_defaults.y_min),
        y_max=_as_float(workspace_raw.get("yMax"), bounds_defaults.y_max),
        z_min=_as_float(workspace_raw.get("zMin"), bounds_defaults.z_min),
        z_max=_as_float(workspace_raw.get("zMax"), bounds_defaults.z_max),
    )

    return VisionConfig(
        esp_base_url=str(raw.get("espBaseUrl", defaults.esp_base_url)).rstrip("/"),
        motion_enabled=_as_bool(raw.get("motionEnabled"), defaults.motion_enabled),
        min_confidence=_as_float(raw.get("minConfidence"), defaults.min_confidence),
        stale_timeout_sec=_as_float(raw.get("staleTimeoutSec"), defaults.stale_timeout_sec),
        safe_hover_z=_as_float(raw.get("safeHoverZ"), defaults.safe_hover_z),
        grab_offset_z=_as_float(raw.get("grabOffsetZ"), defaults.grab_offset_z),
        lift_z=_as_float(raw.get("liftZ"), defaults.lift_z),
        close_claw_degrees=_as_int(raw.get("closeClawDegrees"), defaults.close_claw_degrees),
        move_duration_ms=_as_int(raw.get("moveDurationMs"), defaults.move_duration_ms),
        grab_wait_after_ms=_as_int(raw.get("grabWaitAfterMs"), defaults.grab_wait_after_ms),
        workspace=workspace,
    )
