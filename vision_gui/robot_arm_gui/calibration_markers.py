from __future__ import annotations

import json
from pathlib import Path
from typing import Any


MARKER_DEFINITIONS: dict[int, dict[str, Any]] = {
    0: {"label": "front-left", "short": "FL", "robot": {"x": -150.0, "y": 100.0}},
    1: {"label": "front-right", "short": "FR", "robot": {"x": 150.0, "y": 100.0}},
    2: {"label": "back-left", "short": "BL", "robot": {"x": -150.0, "y": 260.0}},
    3: {"label": "back-right", "short": "BR", "robot": {"x": 150.0, "y": 260.0}},
}


LABEL_TO_ID = {definition["label"]: marker_id for marker_id, definition in MARKER_DEFINITIONS.items()}


def aruco_status() -> tuple[bool, str]:
    try:
        import cv2
    except Exception as error:
        return False, f"OpenCV is not installed: {error}"
    if not hasattr(cv2, "aruco"):
        return False, "OpenCV ArUco support is unavailable. Install opencv-contrib-python."
    aruco = cv2.aruco
    required = ("DICT_4X4_50", "getPredefinedDictionary", "detectMarkers")
    missing = [name for name in required if not hasattr(aruco, name)]
    if missing:
        return False, f"OpenCV ArUco support is incomplete: missing {', '.join(missing)}."
    if not (hasattr(aruco, "generateImageMarker") or hasattr(aruco, "drawMarker")):
        return False, "OpenCV ArUco support cannot generate printable markers."
    return True, "ArUco markers available."


def generate_aruco_marker_sheet(path: Path) -> None:
    cv2, aruco, dictionary = _aruco_context()
    image, draw = _blank_print_sheet("Robot Arm ArUco Calibration Markers")
    positions = _sheet_positions()
    for marker_id, definition in MARKER_DEFINITIONS.items():
        marker = _aruco_marker_image(cv2, aruco, dictionary, marker_id, 640)
        _paste_marker_card(image, draw, positions[marker_id], marker, marker_id, definition, "ArUco ID")
    _save_print_sheet(image, path)


def generate_qr_marker_sheet(path: Path) -> None:
    try:
        import qrcode
    except Exception as error:
        raise RuntimeError(f"QR generation requires qrcode[pil]: {error}") from error
    image, draw = _blank_print_sheet("Robot Arm QR Calibration Markers")
    positions = _sheet_positions()
    for marker_id, definition in MARKER_DEFINITIONS.items():
        payload = {
            "type": "robot_arm_calibration_marker",
            "version": 1,
            "id": marker_id,
            "label": definition["label"],
            "short": definition["short"],
            "robot": definition["robot"],
        }
        qr = qrcode.QRCode(border=2, box_size=16)
        qr.add_data(json.dumps(payload, separators=(",", ":")))
        qr.make(fit=True)
        marker = qr.make_image(fill_color="black", back_color="white").convert("RGB").resize((640, 640))
        _paste_marker_card(image, draw, positions[marker_id], marker, marker_id, definition, "QR ID")
    _save_print_sheet(image, path)


def detect_aruco_markers(frame: Any) -> dict[str, Any]:
    cv2, _aruco, dictionary = _aruco_context()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    parameters = _aruco_parameters(cv2)
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)
    markers: list[dict[str, Any]] = []
    seen: set[int] = set()
    if ids is not None:
        for index, marker_id_value in enumerate(ids.flatten().tolist()):
            marker_id = int(marker_id_value)
            if marker_id not in MARKER_DEFINITIONS:
                continue
            if marker_id in seen:
                continue
            seen.add(marker_id)
            marker_corners = corners[index].reshape(4, 2)
            center_x = float(marker_corners[:, 0].mean())
            center_y = float(marker_corners[:, 1].mean())
            definition = MARKER_DEFINITIONS[marker_id]
            markers.append(
                {
                    "id": marker_id,
                    "label": definition["label"],
                    "short": definition["short"],
                    "pixel": {"x": center_x, "y": center_y},
                    "robot": dict(definition["robot"]),
                    "corners": [{"x": float(x), "y": float(y)} for x, y in marker_corners.tolist()],
                    "source": "aruco",
                }
            )
    warnings = mapping_warnings(markers)
    missing = _missing_labels(markers)
    if missing:
        warnings.append(f"Missing markers: {', '.join(missing)}")
    return {
        "markers": markers,
        "warnings": warnings,
        "rejectedCount": len(rejected) if rejected is not None else 0,
    }


def detect_qr_markers(frame: Any) -> dict[str, Any]:
    try:
        import cv2
    except Exception as error:
        raise RuntimeError(f"QR detection requires OpenCV: {error}") from error
    detector = cv2.QRCodeDetector()
    ok, decoded, points, _straight = detector.detectAndDecodeMulti(frame)
    markers: list[dict[str, Any]] = []
    if ok and points is not None:
        for payload_text, corners in zip(decoded, points, strict=False):
            marker = _parse_qr_payload(payload_text, corners)
            if marker:
                markers.append(marker)
    warnings = mapping_warnings(markers)
    missing = _missing_labels(markers)
    if missing:
        warnings.append(f"Missing markers: {', '.join(missing)}")
    return {"markers": markers, "warnings": warnings}


def mapping_warnings(markers: list[dict[str, Any]]) -> list[str]:
    warnings: list[str] = []
    by_label = {marker.get("label"): marker for marker in markers}
    for left, right in (("front-left", "front-right"), ("back-left", "back-right")):
        if left in by_label and right in by_label and _pixel_x(by_label[left]) >= _pixel_x(by_label[right]):
            warnings.append(f"{_short(left)} appears to the right of {_short(right)}; check for flipped marker placement.")
    for back, front in (("back-left", "front-left"), ("back-right", "front-right")):
        if back in by_label and front in by_label and _pixel_y(by_label[back]) >= _pixel_y(by_label[front]):
            warnings.append(f"{_short(back)} appears below {_short(front)}; check camera/table orientation.")
    labels = list(by_label)
    for index, first in enumerate(labels):
        for second in labels[index + 1 :]:
            dx = _pixel_x(by_label[first]) - _pixel_x(by_label[second])
            dy = _pixel_y(by_label[first]) - _pixel_y(by_label[second])
            if (dx * dx + dy * dy) ** 0.5 < 25.0:
                warnings.append(f"{_short(first)} and {_short(second)} are too close together.")
    return warnings


def marker_definition(label: str) -> dict[str, Any] | None:
    marker_id = LABEL_TO_ID.get(label)
    if marker_id is None:
        return None
    return MARKER_DEFINITIONS[marker_id]


def _aruco_context() -> tuple[Any, Any, Any]:
    try:
        import cv2
    except Exception as error:
        raise RuntimeError(f"OpenCV is not installed: {error}") from error
    ok, message = aruco_status()
    if not ok:
        raise RuntimeError(message)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    return cv2, cv2.aruco, dictionary


def _aruco_parameters(cv2: Any) -> Any:
    if hasattr(cv2.aruco, "DetectorParameters"):
        return cv2.aruco.DetectorParameters()
    return cv2.aruco.DetectorParameters_create()


def _aruco_marker_image(cv2: Any, aruco: Any, dictionary: Any, marker_id: int, size: int) -> Any:
    from PIL import Image

    if hasattr(aruco, "generateImageMarker"):
        marker = aruco.generateImageMarker(dictionary, marker_id, size)
    else:
        import numpy as np

        marker = np.zeros((size, size), dtype="uint8")
        aruco.drawMarker(dictionary, marker_id, size, marker, 1)
    marker = cv2.cvtColor(marker, cv2.COLOR_GRAY2RGB)
    return Image.fromarray(marker)


def _blank_print_sheet(title: str) -> tuple[Any, Any]:
    try:
        from PIL import Image, ImageDraw
    except Exception as error:
        raise RuntimeError(f"Marker sheet generation requires Pillow: {error}") from error
    image = Image.new("RGB", (2550, 3300), "white")
    draw = ImageDraw.Draw(image)
    draw.text((120, 80), title, fill="black")
    draw.text((120, 130), "Print at 100% scale. Place one marker at each known table point.", fill="black")
    return image, draw


def _sheet_positions() -> dict[int, tuple[int, int]]:
    return {
        0: (150, 260),
        1: (1380, 260),
        2: (150, 1760),
        3: (1380, 1760),
    }


def _paste_marker_card(image: Any, draw: Any, origin: tuple[int, int], marker_image: Any, marker_id: int, definition: dict[str, Any], id_label: str) -> None:
    x, y = origin
    image.paste(marker_image, (x, y))
    short = definition["short"]
    label = definition["label"]
    robot = definition["robot"]
    draw.rectangle((x - 35, y - 35, x + 700, y + 860), outline="black", width=4)
    draw.text((x, y + 675), f"{short}  {label}", fill="black")
    draw.text((x, y + 720), f"{id_label}: {marker_id}", fill="black")
    draw.text((x, y + 765), f"Robot X {robot['x']:.0f} mm   Robot Y {robot['y']:.0f} mm", fill="black")


def _save_print_sheet(image: Any, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    suffix = path.suffix.lower()
    if suffix == ".pdf":
        image.save(path, "PDF", resolution=300.0)
    else:
        image.save(path)


def _parse_qr_payload(payload_text: str, corners: Any) -> dict[str, Any] | None:
    try:
        payload = json.loads(payload_text)
    except Exception:
        return None
    if payload.get("type") != "robot_arm_calibration_marker":
        return None
    label = payload.get("label")
    definition = marker_definition(str(label))
    if not definition:
        return None
    marker_id = LABEL_TO_ID[str(label)]
    corner_list = corners.reshape(4, 2)
    center_x = float(corner_list[:, 0].mean())
    center_y = float(corner_list[:, 1].mean())
    return {
        "id": marker_id,
        "label": definition["label"],
        "short": definition["short"],
        "pixel": {"x": center_x, "y": center_y},
        "robot": dict(payload.get("robot") or definition["robot"]),
        "corners": [{"x": float(x), "y": float(y)} for x, y in corner_list.tolist()],
        "source": "qr",
    }


def _missing_labels(markers: list[dict[str, Any]]) -> list[str]:
    found = {marker.get("label") for marker in markers}
    return [definition["short"] for definition in MARKER_DEFINITIONS.values() if definition["label"] not in found]


def _short(label: str) -> str:
    definition = marker_definition(label)
    return str(definition["short"] if definition else label)


def _pixel_x(marker: dict[str, Any]) -> float:
    pixel = marker.get("pixel", {})
    return float(pixel.get("x", 0.0))


def _pixel_y(marker: dict[str, Any]) -> float:
    pixel = marker.get("pixel", {})
    return float(pixel.get("y", 0.0))
