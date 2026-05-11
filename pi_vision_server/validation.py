from __future__ import annotations

import math
from datetime import UTC, datetime
from typing import Any


class ValidationError(ValueError):
    pass


def parse_received_at(value: str) -> datetime:
    return datetime.fromisoformat(value.replace("Z", "+00:00")).astimezone(UTC)


def target_age_sec(target: dict[str, Any], now: datetime | None = None) -> float | None:
    received_at = target.get("receivedAt")
    if not isinstance(received_at, str):
        return None
    now = now or datetime.now(UTC)
    return max(0.0, (now - parse_received_at(received_at)).total_seconds())


def validate_target_payload(payload: Any) -> dict[str, Any]:
    if not isinstance(payload, dict):
        raise ValidationError("payload must be a JSON object")

    target_type = payload.get("type")
    if target_type != "object_detected":
        raise ValidationError("type must be object_detected")

    object_name = payload.get("object")
    if not isinstance(object_name, str) or not object_name.strip():
        raise ValidationError("object must be a non-empty string")

    pixel_x = _required_number(payload, "pixelX")
    pixel_y = _required_number(payload, "pixelY")
    if pixel_x < 0 or pixel_x > 10000 or pixel_y < 0 or pixel_y > 10000:
        raise ValidationError("pixel coordinates are outside accepted camera limits")

    confidence = _required_number(payload, "confidence")
    if confidence < 0.0 or confidence > 1.0:
        raise ValidationError("confidence must be between 0 and 1")

    target: dict[str, Any] = {
        "type": "object_detected",
        "object": object_name.strip()[:80],
        "pixelX": int(round(pixel_x)),
        "pixelY": int(round(pixel_y)),
        "confidence": float(confidence),
    }

    for key in ("robotX", "robotY", "robotZ"):
        value = payload.get(key)
        if value is None:
            continue
        if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
            raise ValidationError(f"{key} must be a finite number")
        if abs(float(value)) > 10000:
            raise ValidationError(f"{key} is outside accepted robot coordinate limits")
        target[key] = float(value)

    return target


def _required_number(payload: dict[str, Any], key: str) -> float:
    value = payload.get(key)
    if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise ValidationError(f"{key} must be a finite number")
    return float(value)

