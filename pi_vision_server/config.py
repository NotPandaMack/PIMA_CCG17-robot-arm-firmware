from __future__ import annotations

import json
import math
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


CONFIG_KEYS = {
    "espBaseUrl",
    "motionEnabled",
    "minConfidence",
    "staleTimeoutSec",
    "safeHoverZ",
    "grabOffsetZ",
    "liftZ",
    "closeClawDegrees",
    "moveDurationMs",
    "grabWaitAfterMs",
    "workspace",
}

WORKSPACE_KEYS = {"xMin", "xMax", "yMin", "yMax", "zMin", "zMax"}


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


def config_to_dict(config: VisionConfig) -> dict[str, Any]:
    return {
        "espBaseUrl": config.esp_base_url,
        "motionEnabled": config.motion_enabled,
        "minConfidence": config.min_confidence,
        "staleTimeoutSec": config.stale_timeout_sec,
        "safeHoverZ": config.safe_hover_z,
        "grabOffsetZ": config.grab_offset_z,
        "liftZ": config.lift_z,
        "closeClawDegrees": config.close_claw_degrees,
        "moveDurationMs": config.move_duration_ms,
        "grabWaitAfterMs": config.grab_wait_after_ms,
        "workspace": {
            "xMin": config.workspace.x_min,
            "xMax": config.workspace.x_max,
            "yMin": config.workspace.y_min,
            "yMax": config.workspace.y_max,
            "zMin": config.workspace.z_min,
            "zMax": config.workspace.z_max,
        },
    }


def save_config(config: VisionConfig, path: Path = DEFAULT_CONFIG_PATH) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as file:
        json.dump(config_to_dict(config), file, indent=2, sort_keys=True)
        file.write("\n")


def update_config(payload: dict[str, Any], path: Path = DEFAULT_CONFIG_PATH) -> VisionConfig:
    if not isinstance(payload, dict):
        raise ValueError("config must be a JSON object")

    unknown = sorted(set(payload) - CONFIG_KEYS)
    if unknown:
        raise ValueError(f"unsupported config field: {unknown[0]}")

    raw = config_to_dict(load_config(path))
    for key, value in payload.items():
        if key == "workspace":
            if not isinstance(value, dict):
                raise ValueError("workspace must be a JSON object")
            unknown_workspace = sorted(set(value) - WORKSPACE_KEYS)
            if unknown_workspace:
                raise ValueError(f"unsupported workspace field: {unknown_workspace[0]}")
            raw["workspace"].update(value)
        else:
            raw[key] = value

    config = _strict_config(raw)
    save_config(config, path)
    return config


def _strict_config(raw: dict[str, Any]) -> VisionConfig:
    esp_base_url = raw.get("espBaseUrl")
    if not isinstance(esp_base_url, str) or not esp_base_url.strip():
        raise ValueError("espBaseUrl must be a non-empty string")

    motion_enabled = raw.get("motionEnabled")
    if not isinstance(motion_enabled, bool):
        raise ValueError("motionEnabled must be true or false")

    workspace_raw = raw.get("workspace")
    if not isinstance(workspace_raw, dict):
        raise ValueError("workspace must be a JSON object")

    workspace = WorkspaceBounds(
        x_min=_strict_float(workspace_raw.get("xMin"), "workspace.xMin"),
        x_max=_strict_float(workspace_raw.get("xMax"), "workspace.xMax"),
        y_min=_strict_float(workspace_raw.get("yMin"), "workspace.yMin"),
        y_max=_strict_float(workspace_raw.get("yMax"), "workspace.yMax"),
        z_min=_strict_float(workspace_raw.get("zMin"), "workspace.zMin"),
        z_max=_strict_float(workspace_raw.get("zMax"), "workspace.zMax"),
    )
    if workspace.x_min >= workspace.x_max:
        raise ValueError("workspace.xMin must be less than workspace.xMax")
    if workspace.y_min >= workspace.y_max:
        raise ValueError("workspace.yMin must be less than workspace.yMax")
    if workspace.z_min >= workspace.z_max:
        raise ValueError("workspace.zMin must be less than workspace.zMax")

    min_confidence = _strict_float(raw.get("minConfidence"), "minConfidence", 0.0, 1.0)
    stale_timeout_sec = _strict_float(raw.get("staleTimeoutSec"), "staleTimeoutSec", 0.1)
    safe_hover_z = _strict_float(raw.get("safeHoverZ"), "safeHoverZ", 0.0)
    grab_offset_z = _strict_float(raw.get("grabOffsetZ"), "grabOffsetZ")
    lift_z = _strict_float(raw.get("liftZ"), "liftZ", 0.0)
    close_claw_degrees = _strict_int(raw.get("closeClawDegrees"), "closeClawDegrees", 0, 180)
    move_duration_ms = _strict_int(raw.get("moveDurationMs"), "moveDurationMs", 1)
    grab_wait_after_ms = _strict_int(raw.get("grabWaitAfterMs"), "grabWaitAfterMs", 0)

    return VisionConfig(
        esp_base_url=esp_base_url.strip().rstrip("/"),
        motion_enabled=motion_enabled,
        min_confidence=min_confidence,
        stale_timeout_sec=stale_timeout_sec,
        safe_hover_z=safe_hover_z,
        grab_offset_z=grab_offset_z,
        lift_z=lift_z,
        close_claw_degrees=close_claw_degrees,
        move_duration_ms=move_duration_ms,
        grab_wait_after_ms=grab_wait_after_ms,
        workspace=workspace,
    )


def _strict_float(value: Any, label: str, minimum: float | None = None, maximum: float | None = None) -> float:
    if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise ValueError(f"{label} must be a finite number")
    number = float(value)
    if minimum is not None and number < minimum:
        raise ValueError(f"{label} must be at least {minimum}")
    if maximum is not None and number > maximum:
        raise ValueError(f"{label} must be at most {maximum}")
    return number


def _strict_int(value: Any, label: str, minimum: int | None = None, maximum: int | None = None) -> int:
    if not isinstance(value, int):
        raise ValueError(f"{label} must be an integer")
    if minimum is not None and value < minimum:
        raise ValueError(f"{label} must be at least {minimum}")
    if maximum is not None and value > maximum:
        raise ValueError(f"{label} must be at most {maximum}")
    return value
