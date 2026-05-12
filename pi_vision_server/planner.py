from __future__ import annotations

import logging
import math
from dataclasses import asdict
from datetime import UTC, datetime
from typing import Any

from .calibration import calibration_status, get_table_z
from .config import VisionConfig
from .validation import target_age_sec


logger = logging.getLogger(__name__)


def build_pick_plan(
    target: dict[str, Any] | None,
    config: VisionConfig,
    calibration: dict[str, Any],
    hover_only: bool = False,
    now: datetime | None = None,
    current_position: dict[str, Any] | None = None,
) -> dict[str, Any]:
    now = now or datetime.now(UTC)
    errors: list[str] = []

    if target is None:
        errors.append("no target is stored")
        return _empty_plan(config, calibration, hover_only, errors)

    age = target_age_sec(target, now)
    if age is None:
        errors.append("target is missing receivedAt")
    elif age > config.stale_timeout_sec:
        errors.append(f"target is stale: {age:.2f}s old")

    confidence = target.get("confidence")
    if not isinstance(confidence, (int, float)) or float(confidence) < config.min_confidence:
        errors.append("target confidence is below minimum")

    robot_x = target.get("robotX")
    robot_y = target.get("robotY")
    if not _finite(robot_x) or not _finite(robot_y):
        errors.append("target must include robotX and robotY")
        return _empty_plan(config, calibration, hover_only, errors, target)

    x = float(robot_x)
    y = float(robot_y)
    table_z = get_table_z(calibration, x, y)
    if table_z is None:
        errors.append("tableZ is missing; complete side-view table height calibration")
        return _empty_plan(config, calibration, hover_only, errors, target)
    z_sign = -1.0 if config.z_axis_inverted else 1.0
    hover_z = table_z + (z_sign * config.safe_hover_z)
    grab_z = table_z + (z_sign * config.grab_offset_z)
    lift_z = table_z + (z_sign * config.lift_z)
    safe_raise_z = table_z + (z_sign * config.pre_approach_raise_mm)

    z_values_hover = [safe_raise_z, hover_z]
    z_values_full = [safe_raise_z, hover_z, grab_z, lift_z]
    bounds_result = workspace_validation(config, x, y, z_values_hover if hover_only else z_values_full)
    if not bounds_result["ok"]:
        errors.extend(bounds_result["errors"])
    collision_result = table_clearance_validation(config, table_z, z_values_hover if hover_only else z_values_full)
    if not collision_result["ok"]:
        errors.extend(collision_result["errors"])

    status = calibration_status(calibration)
    if not status.is_calibrated:
        errors.append("vision calibration is not valid")
    if not status.has_pickup_pitch:
        errors.append("pickupPitchDeg is missing; capture table-skim pitch during calibration")

    pickup_pitch = calibration.get("pickupPitchDeg")
    commands: list[str] = []
    if _finite(pickup_pitch):
        pitch = float(pickup_pitch)
        # Use current position for the pre-approach raise so the arm lifts from wherever it is.
        # Fall back to target coords in preview/test mode when current_position is unavailable.
        if current_position is not None:
            raise_x = float(current_position.get("x", x))
            raise_y = float(current_position.get("y", y))
        else:
            raise_x, raise_y = x, y
        commands = [
            "STOP",
            "CLEAR_TIMELINE",
            "CLAW_OPEN",
            _keyframe("MOVE", raise_x, raise_y, safe_raise_z, pitch, -1, config),
            _keyframe("MOVE", x, y, hover_z, pitch, -1, config),
        ]
        if not hover_only:
            commands.extend(
                [
                    _keyframe("MOVE", x, y, grab_z, pitch, -1, config),
                    _keyframe("GRAB", x, y, grab_z, pitch, config.close_claw_degrees, config),
                    _keyframe("MOVE", x, y, lift_z, pitch, config.close_claw_degrees, config),
                ]
            )
        commands.append("PLAY_REMOTE_TIMELINE")

    plan = {
        "ok": len(errors) == 0,
        "errors": errors,
        "hoverOnly": hover_only,
        "motionEnabled": config.motion_enabled,
        "willSendMotion": len(errors) == 0 and config.motion_enabled,
        "target": target,
        "calculated": {
            "robotX": x,
            "robotY": y,
            "tableZ": table_z,
            "safeRaiseZ": safe_raise_z,
            "hoverZ": hover_z,
            "skimGrabZ": grab_z,
            "liftZ": lift_z,
            "zAxisInverted": config.z_axis_inverted,
            "pickupPitchDeg": float(pickup_pitch) if _finite(pickup_pitch) else None,
            "targetAgeSec": age,
        },
        "workspace": bounds_result,
        "tableClearance": collision_result,
        "calibration": {
            "isCalibrated": status.is_calibrated,
            "hasHomography": status.has_homography,
            "hasPickupPitch": status.has_pickup_pitch,
            "tableZStatus": status.table_z_status,
        },
        "commands": commands,
    }
    return plan


def execute_plan(plan: dict[str, Any], esp_client: Any, esp_status: dict[str, Any] | None = None) -> dict[str, Any]:
    logger.info("Vision pickup plan")
    for index, command in enumerate(plan.get("commands", []), start=1):
        logger.info("%02d %s", index, command)

    if not plan.get("ok"):
        logger.warning("Pickup blocked: %s", "; ".join(plan.get("errors", [])))
        return {**plan, "sent": False}

    if not plan.get("motionEnabled"):
        logger.warning("motionEnabled is false; ghost mode only")
        return {**plan, "sent": False}

    # Use pre-fetched status when available to avoid a redundant network call.
    status = esp_status if esp_status is not None else esp_client.get_status()
    if status.get("estop") is True:
        blocked = {**plan, "ok": False, "sent": False, "errors": [*plan.get("errors", []), "ESP ESTOP is active"]}
        logger.warning("Pickup blocked: ESP ESTOP is active")
        return blocked

    for command in plan.get("commands", []):
        esp_client.send_command(command)
    return {**plan, "sent": True}


def workspace_validation(config: VisionConfig, x: float, y: float, z_values: list[float]) -> dict[str, Any]:
    bounds = config.workspace
    errors = []
    if x < bounds.x_min or x > bounds.x_max:
        errors.append(f"robotX {x:.1f} outside {bounds.x_min:.1f}..{bounds.x_max:.1f}")
    if y < bounds.y_min or y > bounds.y_max:
        errors.append(f"robotY {y:.1f} outside {bounds.y_min:.1f}..{bounds.y_max:.1f}")
    for z in z_values:
        if z < bounds.z_min or z > bounds.z_max:
            errors.append(f"Z {z:.1f} outside {bounds.z_min:.1f}..{bounds.z_max:.1f}")

    return {"ok": len(errors) == 0, "errors": errors, "bounds": asdict(bounds)}


def table_clearance_validation(config: VisionConfig, table_z: float, z_values: list[float]) -> dict[str, Any]:
    errors = []
    for z in z_values:
        if config.z_axis_inverted:
            if z >= table_z:
                errors.append(f"Z {z:.1f} is at or below tableZ {table_z:.1f}")
        elif z <= table_z:
            errors.append(f"Z {z:.1f} is at or below tableZ {table_z:.1f}")
    return {"ok": len(errors) == 0, "errors": errors, "tableZ": table_z, "zAxisInverted": config.z_axis_inverted}


def _keyframe(
    keyframe_type: str,
    x: float,
    y: float,
    z: float,
    pitch: float,
    claw_degrees: int,
    config: VisionConfig,
) -> str:
    return ":".join(
        [
            "ADD_KEYFRAME",
            keyframe_type,
            _fmt(x),
            _fmt(y),
            _fmt(z),
            _fmt(pitch),
            "0",
            str(int(claw_degrees)),
            str(config.move_duration_ms),
            str(config.grab_wait_after_ms if keyframe_type == "GRAB" else 200),
        ]
    )


def _fmt(value: float) -> str:
    return f"{value:.1f}"


def _finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def _empty_plan(
    config: VisionConfig,
    calibration: dict[str, Any],
    hover_only: bool,
    errors: list[str],
    target: dict[str, Any] | None = None,
) -> dict[str, Any]:
    status = calibration_status(calibration)
    return {
        "ok": False,
        "errors": errors,
        "hoverOnly": hover_only,
        "motionEnabled": config.motion_enabled,
        "willSendMotion": False,
        "target": target,
        "calculated": {
            "robotX": target.get("robotX") if target else None,
            "robotY": target.get("robotY") if target else None,
            "safeRaiseZ": None,
            "hoverZ": None,
            "skimGrabZ": None,
            "tableZ": None,
            "zAxisInverted": config.z_axis_inverted,
            "pickupPitchDeg": calibration.get("pickupPitchDeg"),
        },
        "workspace": {"ok": False, "errors": errors, "bounds": asdict(config.workspace)},
        "tableClearance": {"ok": False, "errors": errors, "tableZ": None, "zAxisInverted": config.z_axis_inverted},
        "calibration": {
            "isCalibrated": status.is_calibrated,
            "hasHomography": status.has_homography,
            "hasPickupPitch": status.has_pickup_pitch,
            "tableZStatus": status.table_z_status,
        },
        "commands": [],
    }
