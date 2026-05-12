from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .config import REPO_ROOT


DEFAULT_CALIBRATION_PATH = REPO_ROOT / "config" / "vision_calibration.json"


@dataclass(frozen=True)
class CalibrationStatus:
    is_calibrated: bool
    has_homography: bool
    has_pickup_pitch: bool
    table_z_status: str


def empty_calibration() -> dict[str, Any]:
    return {
        "status": "not_calibrated",
        "createdAt": None,
        "camera": {"width": None, "height": None},
        "origin": None,
        "pickupPitchDeg": None,
        "homography": [],
        "points": [],
        "reprojectionError": None,
        "tableZ": {"method": "placeholder", "points": []},
    }


def load_calibration(path: Path = DEFAULT_CALIBRATION_PATH) -> dict[str, Any]:
    if not path.exists():
        return empty_calibration()

    with path.open("r", encoding="utf-8") as file:
        data = json.load(file)

    merged = empty_calibration()
    merged.update(data)
    if not isinstance(merged.get("tableZ"), dict):
        merged["tableZ"] = {"method": "placeholder", "points": []}
    return merged


def save_calibration(calibration: dict[str, Any], path: Path = DEFAULT_CALIBRATION_PATH) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as file:
        json.dump(calibration, file, indent=2, sort_keys=True)
        file.write("\n")


def reset_calibration(path: Path = DEFAULT_CALIBRATION_PATH) -> dict[str, Any]:
    calibration = empty_calibration()
    save_calibration(calibration, path)
    return calibration


def calibration_status(calibration: dict[str, Any]) -> CalibrationStatus:
    homography = calibration.get("homography")
    has_homography = (
        isinstance(homography, list)
        and len(homography) == 3
        and all(isinstance(row, list) and len(row) == 3 for row in homography)
    )
    pickup_pitch = calibration.get("pickupPitchDeg")
    table_z = calibration.get("tableZ") if isinstance(calibration.get("tableZ"), dict) else {}
    table_points = table_z.get("points", [])
    table_z_status = "placeholder"
    if table_z.get("method") == "side_view_visual_fit" and _finite(table_z.get("z")):
        table_z_status = "calibrated"
    elif isinstance(table_points, list) and len(table_points) >= 3:
        table_z_status = "calibrated"

    return CalibrationStatus(
        is_calibrated=calibration.get("status") == "calibrated" and has_homography,
        has_homography=has_homography,
        has_pickup_pitch=isinstance(pickup_pitch, (int, float)) and math.isfinite(float(pickup_pitch)),
        table_z_status=table_z_status,
    )


def get_table_z(calibration: dict[str, Any], x: float, y: float) -> float:
    table_z = calibration.get("tableZ") if isinstance(calibration.get("tableZ"), dict) else {}
    if table_z.get("method") == "side_view_visual_fit" and _finite(table_z.get("z")):
        return float(table_z["z"])
    points = table_z.get("points", [])
    if not isinstance(points, list) or len(points) < 3:
        return 0.0

    usable = []
    for point in points:
        if not isinstance(point, dict):
            continue
        px = point.get("x")
        py = point.get("y")
        pz = point.get("z")
        if all(isinstance(v, (int, float)) and math.isfinite(float(v)) for v in (px, py, pz)):
            usable.append((float(px), float(py), float(pz)))

    if len(usable) < 3:
        if usable:
            return sum(point[2] for point in usable) / len(usable)
        return 0.0

    fit = _fit_plane(usable)
    if fit is None:
        return sum(point[2] for point in usable) / len(usable)

    a, b, c = fit
    return (a * x) + (b * y) + c


def pixel_to_robot_homography(calibration: dict[str, Any], pixel_x: float, pixel_y: float) -> tuple[float, float] | None:
    homography = calibration.get("homography")
    if (
        not isinstance(homography, list)
        or len(homography) != 3
        or not all(isinstance(row, list) and len(row) == 3 for row in homography)
    ):
        return None

    h = [[float(value) for value in row] for row in homography]
    denom = (h[2][0] * pixel_x) + (h[2][1] * pixel_y) + h[2][2]
    if abs(denom) < 1e-9:
        return None

    robot_x = ((h[0][0] * pixel_x) + (h[0][1] * pixel_y) + h[0][2]) / denom
    robot_y = ((h[1][0] * pixel_x) + (h[1][1] * pixel_y) + h[1][2]) / denom
    return robot_x, robot_y


def _fit_plane(points: list[tuple[float, float, float]]) -> tuple[float, float, float] | None:
    n = float(len(points))
    sx = sum(p[0] for p in points)
    sy = sum(p[1] for p in points)
    sz = sum(p[2] for p in points)
    sxx = sum(p[0] * p[0] for p in points)
    syy = sum(p[1] * p[1] for p in points)
    sxy = sum(p[0] * p[1] for p in points)
    sxz = sum(p[0] * p[2] for p in points)
    syz = sum(p[1] * p[2] for p in points)

    matrix = [
        [sxx, sxy, sx, sxz],
        [sxy, syy, sy, syz],
        [sx, sy, n, sz],
    ]
    return _solve_3x3(matrix)


def _solve_3x3(matrix: list[list[float]]) -> tuple[float, float, float] | None:
    rows = [row[:] for row in matrix]

    for pivot_index in range(3):
        pivot_row = max(range(pivot_index, 3), key=lambda row_index: abs(rows[row_index][pivot_index]))
        if abs(rows[pivot_row][pivot_index]) < 1e-9:
            return None
        rows[pivot_index], rows[pivot_row] = rows[pivot_row], rows[pivot_index]

        pivot = rows[pivot_index][pivot_index]
        for column in range(pivot_index, 4):
            rows[pivot_index][column] /= pivot

        for row_index in range(3):
            if row_index == pivot_index:
                continue
            factor = rows[row_index][pivot_index]
            for column in range(pivot_index, 4):
                rows[row_index][column] -= factor * rows[pivot_index][column]

    return rows[0][3], rows[1][3], rows[2][3]


def _finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))
