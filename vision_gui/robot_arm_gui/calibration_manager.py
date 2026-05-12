from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from .config import DEFAULT_WORKSPACE, LOCAL_CALIBRATION_PATH


CHECKLIST_ITEMS: tuple[tuple[str, str], ...] = (
    ("pi_connected", "Pi server connected"),
    ("esp_connected", "ESP connected"),
    ("webcam_connected", "2D webcam connected"),
    ("object_detection_working", "Object detection working"),
    ("camera_calibration_complete", "Camera calibration complete"),
    ("workspace_bounds_saved", "Workspace bounds saved"),
    ("table_z_calibrated", "Table Z calibrated"),
    ("pickup_pose_calibrated", "Pickup pose calibrated"),
    ("ghost_preview_passed", "Ghost preview passed"),
    ("hover_only_movement_passed", "Hover-only movement passed"),
    ("motion_enabled", "Motion enabled"),
    ("full_pickup_ready", "Full pickup ready"),
)


@dataclass(frozen=True)
class ChecklistState:
    pi_connected: bool = False
    esp_connected: bool = False
    webcam_connected: bool = False
    object_detection_working: bool = False
    camera_calibration_complete: bool = False
    workspace_bounds_saved: bool = False
    table_z_calibrated: bool = False
    pickup_pose_calibrated: bool = False
    ghost_preview_passed: bool = False
    hover_only_movement_passed: bool = False
    motion_enabled: bool = False
    target_valid: bool = False
    estop_inactive: bool = True

    @property
    def full_pickup_ready(self) -> bool:
        return (
            self.pi_connected
            and self.esp_connected
            and self.webcam_connected
            and self.object_detection_working
            and self.camera_calibration_complete
            and self.workspace_bounds_saved
            and self.table_z_calibrated
            and self.pickup_pose_calibrated
            and self.ghost_preview_passed
            and self.hover_only_movement_passed
            and self.motion_enabled
            and self.target_valid
            and self.estop_inactive
        )

    def as_dict(self) -> dict[str, bool]:
        data = asdict(self)
        data["full_pickup_ready"] = self.full_pickup_ready
        return data


def empty_calibration() -> dict[str, Any]:
    return {
        "status": "not_calibrated",
        "createdAt": None,
        "zAxisInverted": True,
        "camera": {"type": "overhead_2d_webcam", "width": None, "height": None},
        "origin": None,
        "pickupPitchDeg": None,
        "skimZ": None,
        "grabOffsetZ": None,
        "hoverZ": None,
        "liftZ": None,
        "clawOpenValue": 0,
        "clawClosedValue": None,
        "workspaceBounds": dict(DEFAULT_WORKSPACE),
        "homography": [],
        "points": [],
        "reprojectionError": None,
        "tableZ": {"method": "placeholder", "points": []},
        "safeHoverZ": None,
        "lowApproachZ": None,
        "minimumClearanceMm": 10.0,
        "hoverClearanceMm": 60.0,
        "approachClearanceMm": 15.0,
        "liftClearanceMm": 90.0,
    }


def load_local_calibration(path: Path = LOCAL_CALIBRATION_PATH) -> dict[str, Any]:
    calibration = empty_calibration()
    if path.exists():
        with path.open("r", encoding="utf-8") as file:
            raw = json.load(file)
        if isinstance(raw, dict):
            calibration.update(raw)
    if not isinstance(calibration.get("workspaceBounds"), dict):
        calibration["workspaceBounds"] = dict(DEFAULT_WORKSPACE)
    if not isinstance(calibration.get("tableZ"), dict):
        calibration["tableZ"] = {"method": "placeholder", "points": []}
    return calibration


def save_local_calibration(calibration: dict[str, Any], path: Path = LOCAL_CALIBRATION_PATH) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as file:
        json.dump(calibration, file, indent=2, sort_keys=True)
        file.write("\n")


def has_homography(calibration: dict[str, Any]) -> bool:
    homography = calibration.get("homography")
    return (
        isinstance(homography, list)
        and len(homography) == 3
        and all(isinstance(row, list) and len(row) == 3 and all(_finite(v) for v in row) for row in homography)
    )


def workspace_bounds_saved(calibration: dict[str, Any]) -> bool:
    bounds = calibration.get("workspaceBounds")
    if not isinstance(bounds, dict):
        return False
    required = ("xMin", "xMax", "yMin", "yMax", "zMin", "zMax")
    if not all(_finite(bounds.get(key)) for key in required):
        return False
    return bounds["xMin"] < bounds["xMax"] and bounds["yMin"] < bounds["yMax"] and bounds["zMin"] < bounds["zMax"]


def table_z_calibrated(calibration: dict[str, Any]) -> bool:
    table_z = calibration.get("tableZ")
    if not isinstance(table_z, dict):
        return False
    if table_z.get("method") in {"side_view_visual_fit", "realsense_depth_plane_anchor_fit"} and _finite(table_z.get("z")):
        return True
    points = table_z.get("points")
    usable = [point for point in points or [] if isinstance(point, dict) and all(_finite(point.get(k)) for k in ("x", "y", "z"))]
    if len(usable) >= 3:
        return True
    return False


def pickup_pose_calibrated(calibration: dict[str, Any]) -> bool:
    return all(
        _finite(calibration.get(key))
        for key in ("pickupPitchDeg", "grabOffsetZ", "hoverZ", "liftZ", "skimZ", "clawOpenValue", "clawClosedValue")
    )


def calibration_complete(calibration: dict[str, Any]) -> bool:
    return has_homography(calibration) and workspace_bounds_saved(calibration) and table_z_calibrated(calibration)


def build_calibration(
    *,
    camera_width: int,
    camera_height: int,
    origin_pixel: tuple[int, int],
    points: list[dict[str, Any]],
    workspace: dict[str, float],
    table_z: dict[str, Any],
    pickup: dict[str, Any],
    z_axis_inverted: bool = True,
) -> dict[str, Any]:
    import cv2
    import numpy as np

    if len(points) != 4:
        raise ValueError("four calibration points are required")
    pixel_points = np.array([[p["pixel"]["x"], p["pixel"]["y"]] for p in points], dtype=np.float32)
    robot_points = np.array([[p["robot"]["x"], p["robot"]["y"]] for p in points], dtype=np.float32)
    validate_point_spacing(points)
    homography, _ = cv2.findHomography(pixel_points, robot_points)
    if homography is None:
        raise ValueError("unable to compute homography")
    reprojection_error = compute_reprojection_error(pixel_points, robot_points, homography)
    calibration = empty_calibration()
    calibration.update(
        {
            "status": "calibrated",
            "createdAt": datetime.now(UTC).isoformat().replace("+00:00", "Z"),
            "zAxisInverted": bool(z_axis_inverted),
            "camera": {"type": "overhead_2d_webcam", "width": int(camera_width), "height": int(camera_height)},
            "origin": {"pixel": {"x": int(origin_pixel[0]), "y": int(origin_pixel[1])}},
            "homography": homography.tolist(),
            "points": points,
            "workspaceBounds": workspace,
            "reprojectionError": reprojection_error,
            "tableZ": table_z,
            **pickup,
        }
    )
    if not calibration_complete(calibration):
        calibration["status"] = "partial"
    return calibration


def build_mapping_homography(points: list[dict[str, Any]]) -> tuple[list[list[float]], float]:
    import cv2
    import numpy as np

    if len(points) != 4:
        raise ValueError("four calibration points are required")
    validate_point_spacing(points)
    pixel_points = np.array([[p["pixel"]["x"], p["pixel"]["y"]] for p in points], dtype=np.float32)
    robot_points = np.array([[p["robot"]["x"], p["robot"]["y"]] for p in points], dtype=np.float32)
    homography, _ = cv2.findHomography(pixel_points, robot_points)
    if homography is None:
        raise ValueError("unable to compute homography")
    return homography.tolist(), compute_reprojection_error(pixel_points, robot_points, homography)


def validate_point_spacing(points: list[dict[str, Any]], min_distance_px: float = 25.0) -> None:
    pixels = [(float(p["pixel"]["x"]), float(p["pixel"]["y"])) for p in points]
    for index, first in enumerate(pixels):
        for second in pixels[index + 1 :]:
            if math.dist(first, second) < min_distance_px:
                raise ValueError("calibration points are duplicated or too close together")


def estimate_table_z(calibration: dict[str, Any], x: float, y: float) -> float:
    import numpy as np

    table_z = calibration.get("tableZ") if isinstance(calibration.get("tableZ"), dict) else {}
    if table_z.get("method") in {"side_view_visual_fit", "realsense_depth_plane_anchor_fit"} and _finite(table_z.get("z")):
        return float(table_z["z"])
    points = table_z.get("points", [])
    usable = []
    for point in points if isinstance(points, list) else []:
        if isinstance(point, dict) and all(_finite(point.get(k)) for k in ("x", "y", "z")):
            usable.append((float(point["x"]), float(point["y"]), float(point["z"])))
    if len(usable) < 3:
        return float(table_z.get("z", 0.0)) if _finite(table_z.get("z")) else 0.0
    matrix = np.array([[px, py, 1.0] for px, py, _ in usable], dtype=np.float64)
    values = np.array([pz for _, _, pz in usable], dtype=np.float64)
    a, b, c = np.linalg.lstsq(matrix, values, rcond=None)[0]
    return float((a * x) + (b * y) + c)


def fit_side_view_table_z(
    *,
    samples: list[dict[str, Any]],
    table_line: dict[str, Any],
    safety_margin_mm: float = 10.0,
    z_axis_inverted: bool = True,
) -> dict[str, Any]:
    usable: list[tuple[float, float, float]] = []
    for sample in samples:
        if not isinstance(sample, dict):
            continue
        pixel = sample.get("pixel")
        if not isinstance(pixel, dict):
            continue
        if _finite(sample.get("robotZ")) and _finite(pixel.get("x")) and _finite(pixel.get("y")):
            usable.append((float(sample["robotZ"]), float(pixel["x"]), float(pixel["y"])))
    if len(usable) < 2:
        raise ValueError("at least two side-view samples are required")

    p1 = table_line.get("p1") if isinstance(table_line, dict) else None
    p2 = table_line.get("p2") if isinstance(table_line, dict) else None
    if not isinstance(p1, dict) or not isinstance(p2, dict):
        raise ValueError("table line requires p1 and p2")
    if not all(_finite(point.get(axis)) for point in (p1, p2) for axis in ("x", "y")):
        raise ValueError("table line points must be finite")
    x1, y1 = float(p1["x"]), float(p1["y"])
    x2, y2 = float(p2["x"]), float(p2["y"])
    if abs(x2 - x1) < 1e-6 and abs(y2 - y1) < 1e-6:
        raise ValueError("table line points must be distinct")

    deltas = [_table_line_y_at_x(x, x1, y1, x2, y2) - y for _robot_z, x, y in usable]
    if max(deltas) - min(deltas) < 1e-6:
        raise ValueError("side-view samples need distinct visual heights")

    n = float(len(usable))
    sum_delta = sum(deltas)
    sum_z = sum(robot_z for robot_z, _x, _y in usable)
    sum_delta_delta = sum(delta * delta for delta in deltas)
    sum_delta_z = sum(delta * robot_z for delta, (robot_z, _x, _y) in zip(deltas, usable, strict=False))
    denom = (n * sum_delta_delta) - (sum_delta * sum_delta)
    if abs(denom) < 1e-9:
        raise ValueError("side-view fit is singular")

    slope_robot_mm_per_px = ((n * sum_delta_z) - (sum_delta * sum_z)) / denom
    if not math.isfinite(slope_robot_mm_per_px):
        raise ValueError("side-view fit is invalid")
    if z_axis_inverted and slope_robot_mm_per_px >= 0.0:
        raise ValueError("inverted-Z samples must have lower robot Z farther above the table line")
    if not z_axis_inverted and slope_robot_mm_per_px <= 0.0:
        raise ValueError("side-view samples must have higher robot Z farther above the table line")
    intercept_table_z = (sum_z - (slope_robot_mm_per_px * sum_delta)) / n
    pixels_per_robot_mm = abs(1.0 / slope_robot_mm_per_px)
    predicted_deltas = [(robot_z - intercept_table_z) * pixels_per_robot_mm for robot_z, _x, _y in usable]
    error_px = sum(abs(actual - predicted) for actual, predicted in zip(deltas, predicted_deltas, strict=False)) / len(deltas)

    return {
        "method": "side_view_visual_fit",
        "zAxisInverted": bool(z_axis_inverted),
        "z": float(intercept_table_z),
        "safetyMarginMm": float(safety_margin_mm),
        "samples": samples,
        "tableLine": table_line,
        "fit": {
            "pixelsPerRobotMm": float(pixels_per_robot_mm),
            "robotMmPerPixel": float(slope_robot_mm_per_px),
            "errorPx": float(error_px),
        },
    }


def table_relative_z_values(
    table_z: float,
    *,
    z_axis_inverted: bool = True,
    hover_clearance_mm: float = 60.0,
    approach_clearance_mm: float = 15.0,
    lift_clearance_mm: float = 90.0,
) -> dict[str, float]:
    sign = -1.0 if z_axis_inverted else 1.0
    return {
        "safeHoverZ": float(table_z + (sign * hover_clearance_mm)),
        "lowApproachZ": float(table_z + (sign * approach_clearance_mm)),
        "liftZ": float(table_z + (sign * lift_clearance_mm)),
    }


def _table_line_y_at_x(x: float, x1: float, y1: float, x2: float, y2: float) -> float:
    if abs(x2 - x1) < 1e-6:
        return (y1 + y2) / 2.0
    t = (x - x1) / (x2 - x1)
    return y1 + (t * (y2 - y1))


def convert_pixel(calibration: dict[str, Any], x: float, y: float) -> tuple[float, float] | None:
    homography = calibration.get("homography")
    if (
        not isinstance(homography, list)
        or len(homography) != 3
        or not all(isinstance(row, list) and len(row) == 3 for row in homography)
    ):
        return None
    h = [[float(value) for value in row] for row in homography]
    denom = (h[2][0] * x) + (h[2][1] * y) + h[2][2]
    if abs(denom) < 1e-9:
        return None
    robot_x = ((h[0][0] * x) + (h[0][1] * y) + h[0][2]) / denom
    robot_y = ((h[1][0] * x) + (h[1][1] * y) + h[1][2]) / denom
    return robot_x, robot_y


def compute_reprojection_error(pixel_points: Any, robot_points: Any, homography: Any) -> float:
    import cv2
    import numpy as np

    projected = cv2.perspectiveTransform(pixel_points.reshape(-1, 1, 2), homography).reshape(-1, 2)
    errors = np.linalg.norm(projected - robot_points, axis=1)
    return float(np.mean(errors))


def _finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))
