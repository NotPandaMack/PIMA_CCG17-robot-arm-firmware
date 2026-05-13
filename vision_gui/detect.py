from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np


@dataclass
class Detection:
    found: bool
    pixel_x: int = 0
    pixel_y: int = 0
    confidence: float = 0.0
    area: float = 0.0
    bounds: tuple[int, int, int, int] | None = None


@dataclass
class HsvRange:
    lower_hue: int = 40
    upper_hue: int = 85
    sat_min: int = 80
    val_min: int = 80
    min_area: float = 350.0

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> "HsvRange":
        return cls(
            lower_hue=int(d.get("lowerHue", 40)),
            upper_hue=int(d.get("upperHue", 85)),
            sat_min=int(d.get("satMin", 80)),
            val_min=int(d.get("valMin", 80)),
            min_area=float(d.get("minArea", 350.0)),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "lowerHue": self.lower_hue,
            "upperHue": self.upper_hue,
            "satMin": self.sat_min,
            "valMin": self.val_min,
            "minArea": self.min_area,
        }


def detect_object(frame: np.ndarray, hsv: HsvRange) -> Detection:
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lo = np.array([hsv.lower_hue, hsv.sat_min, hsv.val_min], dtype=np.uint8)
    hi = np.array([hsv.upper_hue, 255, 255], dtype=np.uint8)
    mask = cv2.inRange(hsv_frame, lo, hi)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return Detection(found=False)

    best = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(best))
    if area < hsv.min_area:
        return Detection(found=False, area=area)

    m = cv2.moments(best)
    if m["m00"] == 0:
        return Detection(found=False, area=area)

    px = int(m["m10"] / m["m00"])
    py = int(m["m01"] / m["m00"])
    frame_area = float(frame.shape[0] * frame.shape[1])
    confidence = max(0.0, min(0.99, 0.65 + min(0.34, area / frame_area * 15.0)))
    bx, by, bw, bh = cv2.boundingRect(best)
    return Detection(found=True, pixel_x=px, pixel_y=py, confidence=confidence, area=area, bounds=(bx, by, bw, bh))


def draw_overlay(frame: np.ndarray, det: Detection, robot_xy: tuple[float, float] | None) -> np.ndarray:
    out = frame.copy()
    if det.found:
        if det.bounds:
            x, y, w, h = det.bounds
            cv2.rectangle(out, (x, y), (x + w, y + h), (80, 255, 80), 2)
        cv2.circle(out, (det.pixel_x, det.pixel_y), 6, (0, 0, 255), -1)
        label = f"conf={det.confidence:.2f}  px=({det.pixel_x},{det.pixel_y})"
        if robot_xy:
            label += f"  robot=({robot_xy[0]:.1f},{robot_xy[1]:.1f})"
        cv2.putText(out, label, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    return out


def detect_aruco_markers(frame: np.ndarray) -> list[dict[str, Any]]:
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)
    corners, ids, _ = detector.detectMarkers(frame)
    if ids is None:
        return []
    results = []
    for corner, marker_id in zip(corners, ids.flatten()):
        cx = float(corner[0][:, 0].mean())
        cy = float(corner[0][:, 1].mean())
        results.append({"id": int(marker_id), "cx": cx, "cy": cy, "corners": corner[0].tolist()})
    return results


def pixel_to_robot(px: float, py: float, homography: list[list[float]]) -> tuple[float, float] | None:
    mat = np.array(homography, dtype=np.float64)
    if mat.shape != (3, 3):
        return None
    pt = np.array([[[px, py]]], dtype=np.float32)
    out = cv2.perspectiveTransform(pt, mat)
    return float(out[0][0][0]), float(out[0][0][1])


def compute_homography(
    pixel_points: list[tuple[float, float]],
    robot_points: list[tuple[float, float]],
) -> list[list[float]] | None:
    if len(pixel_points) < 4 or len(pixel_points) != len(robot_points):
        return None
    src = np.array(pixel_points, dtype=np.float32)
    dst = np.array(robot_points, dtype=np.float32)
    mat, _ = cv2.findHomography(src, dst, cv2.RANSAC, 5.0)
    if mat is None:
        return None
    return mat.tolist()
