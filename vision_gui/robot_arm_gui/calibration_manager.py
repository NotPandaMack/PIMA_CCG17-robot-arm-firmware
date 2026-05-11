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
    ("webcam_connected", "Webcam connected"),
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
        "camera": {"width": None, "height": None},
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
    points = table_z.get("points")
    usable = [point for point in points or [] if isinstance(point, dict) and all(_finite(point.get(k)) for k in ("x", "y", "z"))]
    if len(usable) >= 3:
        return True
    return table_z.get("method") == "placeholder" and _finite(table_z.get("z"))


def pickup_pose_calibrated(calibration: dict[str, Any]) -> bool:
    return all(
        _finite(calibration.get(key))
        for key in ("pickupPitchDeg", "grabOffsetZ", "hoverZ", "liftZ", "skimZ", "clawOpenValue", "clawClosedValue")
    )


def calibration_complete(calibration: dict[str, Any]) -> bool:
    return has_homography(calibration) and workspace_bounds_saved(calibration) and table_z_calibrated(calibration) and pickup_pose_calibrated(calibration)


def build_calibration(
    *,
    camera_width: int,
    camera_height: int,
    origin_pixel: tuple[int, int],
    points: list[dict[str, Any]],
    workspace: dict[str, float],
    table_z: dict[str, Any],
    pickup: dict[str, Any],
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
            "camera": {"width": int(camera_width), "height": int(camera_height)},
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


def validate_point_spacing(points: list[dict[str, Any]], min_distance_px: float = 25.0) -> None:
    pixels = [(float(p["pixel"]["x"]), float(p["pixel"]["y"])) for p in points]
    for index, first in enumerate(pixels):
        for second in pixels[index + 1 :]:
            if math.dist(first, second) < min_distance_px:
                raise ValueError("calibration points are duplicated or too close together")


def estimate_table_z(calibration: dict[str, Any], x: float, y: float) -> float:
    import numpy as np

    table_z = calibration.get("tableZ") if isinstance(calibration.get("tableZ"), dict) else {}
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
