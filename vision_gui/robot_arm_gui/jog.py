from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any


JOG_STEP_MM = 2.0
PITCH_STEP_DEG = 2.0
MIN_MANUAL_CLEARANCE_MM = 5.0


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    z: float
    pitch: float


@dataclass(frozen=True)
class JogPlan:
    pose: Pose
    command: str


def pose_from_status(status: dict[str, Any]) -> Pose:
    values = {
        "x": _required_float(status, "x"),
        "y": _required_float(status, "y"),
        "z": _required_float(status, "z"),
        "pitch": _required_float(status, "pitch"),
    }
    return Pose(values["x"], values["y"], values["z"], values["pitch"])


def build_jog_plan(
    status: dict[str, Any],
    axis: str,
    direction: int,
    *,
    step_mm: float = JOG_STEP_MM,
    pitch_step_deg: float = PITCH_STEP_DEG,
    table_z: float | None = None,
    minimum_clearance: float = MIN_MANUAL_CLEARANCE_MM,
    calibration_touch_mode: bool = False,
) -> JogPlan:
    if direction not in (-1, 1):
        raise ValueError("jog direction must be -1 or 1")
    axis = axis.upper()
    current = pose_from_status(status)
    x, y, z, pitch = current.x, current.y, current.z, current.pitch

    if axis == "X":
        x += direction * step_mm
    elif axis == "Y":
        y += direction * step_mm
    elif axis == "Z":
        z += direction * step_mm
    elif axis == "PITCH":
        pitch += direction * pitch_step_deg
    else:
        raise ValueError(f"unsupported jog axis: {axis}")

    if axis == "Z" and direction < 0:
        if current.z - z > step_mm + 1e-6:
            raise ValueError("Z lowering exceeds the configured small jog step")
        if table_z is not None and not calibration_touch_mode and z < table_z + minimum_clearance:
            raise ValueError("manual jog refused: target Z is below table clearance")
        if calibration_touch_mode and current.z - z > step_mm + 1e-6:
            raise ValueError("calibration lowering mode only allows the small Z step")

    next_pose = Pose(x, y, z, pitch)
    return JogPlan(next_pose, pose_to_set_target(next_pose))


def pose_to_set_target(pose: Pose) -> str:
    return f"SET_TARGET:{_fmt(pose.x)}:{_fmt(pose.y)}:{_fmt(pose.z)}:{_fmt(pose.pitch)}"


def _required_float(status: dict[str, Any], key: str) -> float:
    value = status.get(key)
    if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise ValueError(f"ESP status missing valid {key}")
    return float(value)


def _fmt(value: float) -> str:
    return f"{value:.1f}"
