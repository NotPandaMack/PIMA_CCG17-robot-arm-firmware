from __future__ import annotations

import math
from typing import Any


def sample_depth_mm(depth_image: Any, pixel_x: float, pixel_y: float, *, radius: int = 3, depth_scale: float = 0.001) -> float | None:
    import numpy as np

    if depth_image is None:
        return None
    height, width = depth_image.shape[:2]
    cx = int(round(pixel_x))
    cy = int(round(pixel_y))
    x0 = max(0, cx - radius)
    x1 = min(width, cx + radius + 1)
    y0 = max(0, cy - radius)
    y1 = min(height, cy + radius + 1)
    if x0 >= x1 or y0 >= y1:
        return None
    patch = np.asarray(depth_image[y0:y1, x0:x1], dtype=np.float64).reshape(-1)
    valid = patch[patch > 0.0]
    if valid.size == 0:
        return None
    return float(np.median(valid) * float(depth_scale) * 1000.0)


def deproject_pixel_to_point_mm(pixel_x: float, pixel_y: float, depth_mm: float, intrinsics: dict[str, float]) -> tuple[float, float, float]:
    fx = float(intrinsics["fx"])
    fy = float(intrinsics["fy"])
    ppx = float(intrinsics["ppx"])
    ppy = float(intrinsics["ppy"])
    if abs(fx) < 1e-9 or abs(fy) < 1e-9:
        raise ValueError("RealSense intrinsics have invalid focal length")
    x = ((float(pixel_x) - ppx) / fx) * float(depth_mm)
    y = ((float(pixel_y) - ppy) / fy) * float(depth_mm)
    return float(x), float(y), float(depth_mm)


def fit_table_plane_from_depth(
    *,
    depth_image: Any,
    intrinsics: dict[str, float],
    marker_points: list[dict[str, Any]] | None = None,
    depth_scale: float = 0.001,
    stride: int = 8,
) -> dict[str, Any]:
    import cv2
    import numpy as np

    if depth_image is None:
        raise ValueError("RealSense depth frame is missing")
    height, width = depth_image.shape[:2]
    if marker_points and len(marker_points) >= 4:
        polygon = _marker_polygon(marker_points)
    else:
        margin_x = int(width * 0.15)
        margin_y = int(height * 0.15)
        polygon = [
            (margin_x, margin_y),
            (width - margin_x, margin_y),
            (width - margin_x, height - margin_y),
            (margin_x, height - margin_y),
        ]
    mask = np.zeros((height, width), dtype=np.uint8)
    cv2.fillConvexPoly(mask, np.array(polygon, dtype=np.int32), 255)

    points: list[tuple[float, float, float]] = []
    for y in range(0, height, max(1, int(stride))):
        for x in range(0, width, max(1, int(stride))):
            if mask[y, x] == 0:
                continue
            raw_depth = float(depth_image[y, x])
            if raw_depth <= 0.0:
                continue
            depth_mm = raw_depth * float(depth_scale) * 1000.0
            if not math.isfinite(depth_mm) or depth_mm <= 0.0:
                continue
            points.append(deproject_pixel_to_point_mm(x, y, depth_mm, intrinsics))

    if len(points) < 50:
        raise ValueError("not enough valid RealSense depth points inside the marker area")

    plane = _fit_plane(points)
    distances = [_point_plane_distance(point, plane) for point in points]
    median_distance = float(np.median(np.abs(distances)))
    filtered = [point for point, distance in zip(points, distances, strict=False) if abs(distance) <= max(8.0, median_distance * 3.0)]
    if len(filtered) >= 50:
        plane = _fit_plane(filtered)
        points = filtered
        distances = [_point_plane_distance(point, plane) for point in points]

    rms = math.sqrt(sum(distance * distance for distance in distances) / len(distances))
    return {
        "method": "realsense_depth_plane",
        "plane": {"a": plane[0], "b": plane[1], "c": plane[2], "d": plane[3]},
        "inlierCount": len(points),
        "rmsErrorMm": float(rms),
        "depthFillRatio": float(len(points) / max(1, int(mask.sum() / 255 / (max(1, int(stride)) ** 2)))),
        "markerPolygon": [{"x": int(x), "y": int(y)} for x, y in polygon],
    }


def height_above_table_mm(point_mm: tuple[float, float, float], table_plane: dict[str, Any]) -> float:
    plane = table_plane.get("plane") if isinstance(table_plane, dict) else None
    if not isinstance(plane, dict):
        raise ValueError("table plane is missing")
    coeffs = (float(plane["a"]), float(plane["b"]), float(plane["c"]), float(plane["d"]))
    return abs(_point_plane_distance(point_mm, coeffs))


def fit_realsense_table_z(
    *,
    samples: list[dict[str, Any]],
    table_plane: dict[str, Any],
    safety_margin_mm: float = 10.0,
    z_axis_inverted: bool = True,
) -> dict[str, Any]:
    usable = []
    for sample in samples:
        if not isinstance(sample, dict):
            continue
        robot_z = sample.get("robotZ")
        height_mm = sample.get("heightAboveTableMm")
        if isinstance(robot_z, (int, float)) and isinstance(height_mm, (int, float)) and math.isfinite(float(robot_z)) and math.isfinite(float(height_mm)):
            usable.append((float(robot_z), abs(float(height_mm))))
    if len(usable) < 3:
        raise ValueError("at least three RealSense depth anchor samples are required")
    heights = [height for _robot_z, height in usable]
    if max(heights) - min(heights) < 15.0:
        raise ValueError("RealSense Z samples need at least 15 mm of height spread")

    if z_axis_inverted:
        estimates = [robot_z + height for robot_z, height in usable]
    else:
        estimates = [robot_z - height for robot_z, height in usable]

    # MAD-based outlier rejection — protects against bad depth readings (e.g. height ≈ 0 or 265 mm)
    if len(estimates) >= 3:
        sorted_est = sorted(estimates)
        med = sorted_est[len(sorted_est) // 2]
        mad = sorted([abs(e - med) for e in estimates])[len(estimates) // 2]
        # 3-MAD cutoff, floor of 5 mm to avoid over-filtering tight clusters
        cutoff = max(5.0, mad * 3.0)
        filtered_pairs = [(rz, h) for (rz, h), e in zip(usable, estimates) if abs(e - med) <= cutoff]
        filtered_estimates = [e for e in estimates if abs(e - med) <= cutoff]
        if len(filtered_pairs) >= 2:
            usable = filtered_pairs
            estimates = filtered_estimates

    table_z = sum(estimates) / len(estimates)
    error_mm = sum(abs(estimate - table_z) for estimate in estimates) / len(estimates)
    return {
        "method": "realsense_depth_plane_anchor_fit",
        "zAxisInverted": bool(z_axis_inverted),
        "z": float(table_z),
        "safetyMarginMm": float(safety_margin_mm),
        "samples": samples,
        "tablePlane": table_plane,
        "fit": {"errorMm": float(error_mm), "sampleCount": len(usable)},
    }


def fit_realsense_table_z_two_sample(
    *,
    low_anchor: dict[str, Any],
    high_anchor: dict[str, Any],
    safety_margin_mm: float = 10.0,
) -> dict[str, Any]:
    """Fit table Z from exactly two anchor positions.

    low_anchor: sample captured with the arm near the table
    high_anchor: sample captured with the arm raised high above the table

    z_axis_inverted is auto-detected from the sign relationship between
    delta_robot_z and delta_height_above_table:
    - Inverted (higher Z number = physically lower): physically high arm has SMALLER robot Z
      → delta_robot_z (high - low) < 0 while delta_height > 0 → opposite signs
    - Not inverted: delta_robot_z > 0 and delta_height > 0 → same signs
    """
    low_robot_z, low_h = _extract_anchor_values(low_anchor)
    high_robot_z, high_h = _extract_anchor_values(high_anchor)

    delta_robot_z = high_robot_z - low_robot_z
    delta_height = high_h - low_h

    if abs(delta_height) < 15.0:
        raise ValueError(f"anchors need at least 15 mm of height spread (got {abs(delta_height):.1f} mm) — move arm further from table for HIGH anchor")
    if abs(delta_robot_z) < 5.0:
        raise ValueError(f"anchors need at least 5 mm of Z change in robot coordinates (got {abs(delta_robot_z):.1f} mm)")

    z_axis_inverted = (delta_robot_z * delta_height) < 0

    if z_axis_inverted:
        est_low = low_robot_z + low_h
        est_high = high_robot_z + high_h
    else:
        est_low = low_robot_z - low_h
        est_high = high_robot_z - high_h

    table_z = (est_low + est_high) / 2.0
    error_mm = abs(est_high - est_low) / 2.0
    return {
        "method": "realsense_two_sample",
        "zAxisInverted": bool(z_axis_inverted),
        "z": float(table_z),
        "safetyMarginMm": float(safety_margin_mm),
        "lowAnchor": low_anchor,
        "highAnchor": high_anchor,
        "fit": {
            "errorMm": float(error_mm),
            "estLowMm": float(est_low),
            "estHighMm": float(est_high),
        },
    }


def _extract_anchor_values(anchor: dict[str, Any]) -> tuple[float, float]:
    robot_z = anchor.get("robotZ")
    height_mm = anchor.get("heightAboveTableMm")
    if not isinstance(robot_z, (int, float)) or not isinstance(height_mm, (int, float)):
        raise ValueError("anchor is missing robotZ or heightAboveTableMm")
    if not math.isfinite(float(robot_z)) or not math.isfinite(float(height_mm)):
        raise ValueError("anchor robotZ or heightAboveTableMm is not finite")
    return float(robot_z), abs(float(height_mm))


def _marker_polygon(marker_points: list[dict[str, Any]]) -> list[tuple[int, int]]:
    centers = []
    for marker in marker_points:
        pixel = marker.get("pixel") if isinstance(marker, dict) else None
        if isinstance(pixel, dict) and isinstance(pixel.get("x"), (int, float)) and isinstance(pixel.get("y"), (int, float)):
            centers.append((float(pixel["x"]), float(pixel["y"])))
    if len(centers) < 4:
        raise ValueError("four marker pixel centers are required")
    center_x = sum(point[0] for point in centers) / len(centers)
    center_y = sum(point[1] for point in centers) / len(centers)
    ordered = sorted(centers, key=lambda point: math.atan2(point[1] - center_y, point[0] - center_x))
    return [(int(round(x)), int(round(y))) for x, y in ordered]


def _fit_plane(points: list[tuple[float, float, float]]) -> tuple[float, float, float, float]:
    import numpy as np

    array = np.asarray(points, dtype=np.float64)
    centroid = array.mean(axis=0)
    _u, _s, vh = np.linalg.svd(array - centroid, full_matrices=False)
    normal = vh[-1, :]
    norm = float(np.linalg.norm(normal))
    if norm < 1e-9:
        raise ValueError("RealSense depth plane fit is singular")
    normal = normal / norm
    d = -float(normal.dot(centroid))
    return float(normal[0]), float(normal[1]), float(normal[2]), d


def _point_plane_distance(point: tuple[float, float, float], plane: tuple[float, float, float, float]) -> float:
    a, b, c, d = plane
    return (a * float(point[0])) + (b * float(point[1])) + (c * float(point[2])) + d
