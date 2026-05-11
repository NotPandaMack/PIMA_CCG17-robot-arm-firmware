from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any


@dataclass(frozen=True)
class HSVProfile:
    lower_hue: int = 40
    upper_hue: int = 85
    saturation_min: int = 80
    value_min: int = 80
    min_area: float = 350.0

    @classmethod
    def from_settings(cls, data: dict[str, Any]) -> "HSVProfile":
        return cls(
            lower_hue=int(data.get("lowerHue", 40)),
            upper_hue=int(data.get("upperHue", 85)),
            saturation_min=int(data.get("saturationMin", 80)),
            value_min=int(data.get("valueMin", 80)),
            min_area=float(data.get("minArea", 350.0)),
        )

    def to_settings(self) -> dict[str, Any]:
        return {
            "lowerHue": self.lower_hue,
            "upperHue": self.upper_hue,
            "saturationMin": self.saturation_min,
            "valueMin": self.value_min,
            "minArea": self.min_area,
        }


@dataclass(frozen=True)
class DetectionResult:
    found: bool
    pixel_x: int | None = None
    pixel_y: int | None = None
    area: float = 0.0
    confidence: float = 0.0
    bounds: tuple[int, int, int, int] | None = None

    def as_payload(self, robot_xy: tuple[float, float] | None = None) -> dict[str, Any] | None:
        if not self.found or self.pixel_x is None or self.pixel_y is None:
            return None
        payload: dict[str, Any] = {
            "type": "object_detected",
            "object": "green_object",
            "pixelX": int(self.pixel_x),
            "pixelY": int(self.pixel_y),
            "confidence": float(self.confidence),
        }
        if robot_xy is not None:
            payload["robotX"] = float(robot_xy[0])
            payload["robotY"] = float(robot_xy[1])
        return payload

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def detect_green_object(frame: np.ndarray, profile: HSVProfile | None = None) -> DetectionResult:
    import cv2
    import numpy as np

    profile = profile or HSVProfile()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower = np.array([profile.lower_hue, profile.saturation_min, profile.value_min], dtype=np.uint8)
    upper = np.array([profile.upper_hue, 255, 255], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return DetectionResult(found=False)

    contour = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(contour))
    if area < profile.min_area:
        return DetectionResult(found=False, area=area)

    moments = cv2.moments(contour)
    if moments["m00"] == 0:
        return DetectionResult(found=False, area=area)

    x = int(moments["m10"] / moments["m00"])
    y = int(moments["m01"] / moments["m00"])
    bx, by, bw, bh = cv2.boundingRect(contour)
    frame_area = float(frame.shape[0] * frame.shape[1])
    confidence = max(0.0, min(0.99, 0.65 + min(0.34, area / frame_area * 15.0)))
    return DetectionResult(found=True, pixel_x=x, pixel_y=y, area=area, confidence=confidence, bounds=(bx, by, bw, bh))


def draw_detection_overlay(
    frame: np.ndarray,
    detection: DetectionResult,
    robot_xy: tuple[float, float] | None = None,
    calibrated: bool = False,
) -> np.ndarray:
    import cv2

    output = frame.copy()
    if detection.found and detection.pixel_x is not None and detection.pixel_y is not None:
        if detection.bounds is not None:
            x, y, w, h = detection.bounds
            cv2.rectangle(output, (x, y), (x + w, y + h), (80, 255, 80), 2)
        cv2.circle(output, (detection.pixel_x, detection.pixel_y), 5, (0, 0, 255), -1)
        label = f"green {detection.confidence:.2f} px=({detection.pixel_x},{detection.pixel_y})"
        if robot_xy is not None:
            label += f" robot=({robot_xy[0]:.1f},{robot_xy[1]:.1f})"
        cv2.putText(output, label, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2)
    message = "Calibrated robot coordinates available." if calibrated else "Pixels only. Calibration required before robot movement."
    cv2.putText(output, message, (12, output.shape[0] - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
    return output


def pixel_to_robot(pixel_x: float, pixel_y: float, homography: Any) -> tuple[float, float] | None:
    import cv2
    import numpy as np

    matrix = np.array(homography, dtype=np.float64)
    if matrix.shape != (3, 3):
        return None
    point = np.array([[[float(pixel_x), float(pixel_y)]]], dtype=np.float32)
    transformed = cv2.perspectiveTransform(point, matrix)
    return float(transformed[0][0][0]), float(transformed[0][0][1])


def compute_reprojection_error(pixel_points: np.ndarray, robot_points: np.ndarray, homography: np.ndarray) -> float:
    import cv2
    import numpy as np

    projected = cv2.perspectiveTransform(pixel_points.reshape(-1, 1, 2), homography).reshape(-1, 2)
    errors = np.linalg.norm(projected - robot_points, axis=1)
    return float(np.mean(errors))
