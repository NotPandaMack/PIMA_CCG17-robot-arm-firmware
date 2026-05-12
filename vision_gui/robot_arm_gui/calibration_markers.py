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

SIDE_BOARD_MARKER_IDS = tuple(range(20, 32))
CHARUCO_SIDE_BOARD_IDS = tuple(range(20, 37))
CHARUCO_BOARD_SIZE = (7, 5)


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


def detect_side_board_markers(frame: Any) -> dict[str, Any]:
    try:
        import cv2
    except Exception as error:
        raise RuntimeError(f"OpenCV is not installed: {error}") from error
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    markers: list[dict[str, Any]] = []
    rejected_count = 0
    if hasattr(cv2, "aruco"):
        charuco = _detect_charuco_side_board(cv2, gray)
        if charuco:
            return charuco
        try:
            dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            parameters = _aruco_parameters(cv2)
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)
            rejected_count = len(rejected) if rejected is not None else 0
            if ids is not None:
                for index, marker_id_value in enumerate(ids.flatten().tolist()):
                    marker_id = int(marker_id_value)
                    if marker_id not in SIDE_BOARD_MARKER_IDS:
                        continue
                    marker_corners = corners[index].reshape(4, 2)
                    markers.append(_side_marker(marker_id, marker_corners, "side-board-aruco"))
        except Exception:
            markers = []
    if not markers:
        markers = _detect_side_board_blocks(cv2, gray)
    checkerboard = None
    if not markers:
        checkerboard = _detect_checkerboard_side_board(cv2, gray)
        if checkerboard:
            return checkerboard
    warnings = []
    if len(markers) < 3:
        warnings.append("Need at least three side-board markers visible for a stable board check.")
    return {"type": "aruco_grid", "markers": markers, "warnings": warnings, "rejectedCount": rejected_count}


def _detect_charuco_side_board(cv2: Any, gray: Any) -> dict[str, Any] | None:
    import numpy as np

    aruco = cv2.aruco
    required = ("CharucoBoard", "CharucoDetector")
    if any(not hasattr(aruco, name) for name in required):
        return None
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    ids = np.array(CHARUCO_SIDE_BOARD_IDS, dtype=np.int32)
    board = aruco.CharucoBoard(CHARUCO_BOARD_SIZE, 1.0, 0.68, dictionary, ids)
    detector = aruco.CharucoDetector(board)
    try:
        charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(gray)
    except Exception:
        return None
    markers: list[dict[str, Any]] = []
    if marker_ids is not None:
        for index, marker_id_value in enumerate(marker_ids.flatten().tolist()):
            marker_id = int(marker_id_value)
            if marker_id not in CHARUCO_SIDE_BOARD_IDS:
                continue
            markers.append(_side_marker(marker_id, marker_corners[index].reshape(4, 2), "side-board-charuco-aruco"))
    corner_count = 0 if charuco_ids is None else len(charuco_ids)
    if corner_count < 4:
        return None
    chessboard_corners = board.getChessboardCorners()
    image_points = []
    object_points = []
    charuco_points = []
    for corner, corner_id_value in zip(charuco_corners.reshape(-1, 2), charuco_ids.flatten().tolist(), strict=False):
        corner_id = int(corner_id_value)
        object_corner = chessboard_corners[corner_id]
        image_points.append([float(corner[0]), float(corner[1])])
        object_points.append([float(object_corner[0]), float(object_corner[1])])
        charuco_points.append({"id": corner_id, "x": float(corner[0]), "y": float(corner[1])})
    quality = "good" if corner_count >= 12 else "warning" if corner_count >= 6 else "poor"
    reprojection_error = None
    outline = None
    axes = None
    if len(image_points) >= 4:
        homography, _mask = cv2.findHomography(np.array(object_points, dtype=np.float32), np.array(image_points, dtype=np.float32))
        if homography is not None:
            projected = cv2.perspectiveTransform(np.array(object_points, dtype=np.float32).reshape(-1, 1, 2), homography).reshape(-1, 2)
            errors = np.linalg.norm(projected - np.array(image_points, dtype=np.float32), axis=1)
            reprojection_error = float(errors.mean())
            max_x = float(CHARUCO_BOARD_SIZE[0] - 1)
            max_y = float(CHARUCO_BOARD_SIZE[1] - 1)
            outline_points = np.array([[[0.0, 0.0]], [[max_x, 0.0]], [[max_x, max_y]], [[0.0, max_y]]], dtype=np.float32)
            projected_outline = cv2.perspectiveTransform(outline_points, homography).reshape(-1, 2)
            outline = [{"x": float(x), "y": float(y)} for x, y in projected_outline.tolist()]
            axis_points = np.array([[[0.0, 0.0]], [[1.5, 0.0]], [[0.0, 1.5]]], dtype=np.float32)
            projected_axes = cv2.perspectiveTransform(axis_points, homography).reshape(-1, 2)
            axes = {
                "origin": {"x": float(projected_axes[0][0]), "y": float(projected_axes[0][1])},
                "x": {"x": float(projected_axes[1][0]), "y": float(projected_axes[1][1])},
                "y": {"x": float(projected_axes[2][0]), "y": float(projected_axes[2][1])},
            }
    warnings = []
    if corner_count < 6:
        warnings.append("Too few ChArUco corners are visible.")
    elif corner_count < 12:
        warnings.append("Move phone or monitor until more ChArUco corners are visible.")
    return {
        "type": "charuco",
        "markers": markers,
        "charucoCorners": charuco_points,
        "charucoCornerCount": corner_count,
        "visibleArucoIds": [int(value) for value in marker_ids.flatten().tolist()] if marker_ids is not None else [],
        "quality": quality,
        "reprojectionError": reprojection_error,
        "boardOutline": outline,
        "axes": axes,
        "warnings": warnings,
        "rejectedCount": 0,
    }


def _detect_checkerboard_side_board(cv2: Any, gray: Any) -> dict[str, Any] | None:
    ok, corners = cv2.findChessboardCorners(gray, (6, 4))
    if not ok or corners is None:
        return None
    points = corners.reshape(-1, 2)
    outline_indices = (0, 5, 23, 18)
    outline = [{"x": float(points[index][0]), "y": float(points[index][1])} for index in outline_indices]
    return {
        "type": "checkerboard",
        "markers": [],
        "charucoCorners": [{"id": index, "x": float(x), "y": float(y)} for index, (x, y) in enumerate(points.tolist())],
        "charucoCornerCount": len(points),
        "visibleArucoIds": [],
        "quality": "fallback",
        "reprojectionError": None,
        "boardOutline": outline,
        "axes": None,
        "warnings": ["Using checkerboard fallback; ChArUco markers were not detected."],
        "rejectedCount": 0,
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


def _side_marker(marker_id: int, marker_corners: Any, source: str) -> dict[str, Any]:
    center_x = float(marker_corners[:, 0].mean())
    center_y = float(marker_corners[:, 1].mean())
    return {
        "id": marker_id,
        "label": f"side-board-{marker_id}",
        "short": f"S{marker_id}",
        "pixel": {"x": center_x, "y": center_y},
        "corners": [{"x": float(x), "y": float(y)} for x, y in marker_corners.tolist()],
        "source": source,
    }


def _detect_side_board_blocks(cv2: Any, gray: Any) -> list[dict[str, Any]]:
    import numpy as np

    dark_binary = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY_INV)[1]
    light_binary = cv2.threshold(gray, 175, 255, cv2.THRESH_BINARY)[1]
    frame_area = float(gray.shape[0] * gray.shape[1])
    candidates = _side_board_candidates(cv2, dark_binary, frame_area) + _side_board_candidates(cv2, light_binary, frame_area)
    candidates.sort(key=lambda item: (item[1], item[0]))
    markers = []
    for offset, (_x, _y, points) in enumerate(candidates[: len(SIDE_BOARD_MARKER_IDS)]):
        ordered = _order_quad_points(np.array(points, dtype="float32"))
        markers.append(_side_marker(SIDE_BOARD_MARKER_IDS[offset], ordered, "side-board-visual"))
    return markers


def _side_board_candidates(cv2: Any, binary: Any, frame_area: float) -> list[tuple[int, int, Any]]:
    contours, _hierarchy = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < frame_area * 0.002 or area > frame_area * 0.25:
            continue
        perimeter = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
        if len(approx) != 4:
            continue
        points = approx.reshape(4, 2).astype("float32")
        x, y, w, h = cv2.boundingRect(points.astype("int32"))
        if h <= 0 or w <= 0:
            continue
        aspect = w / h
        if aspect < 0.72 or aspect > 1.28:
            continue
        candidates.append((x, y, points))
    return candidates


def _order_quad_points(points: Any) -> Any:
    import numpy as np

    ordered = np.zeros((4, 2), dtype="float32")
    sums = points.sum(axis=1)
    diffs = np.diff(points, axis=1).reshape(4)
    ordered[0] = points[int(np.argmin(sums))]
    ordered[2] = points[int(np.argmax(sums))]
    ordered[1] = points[int(np.argmin(diffs))]
    ordered[3] = points[int(np.argmax(diffs))]
    return ordered


def _short(label: str) -> str:
    definition = marker_definition(label)
    return str(definition["short"] if definition else label)


def _pixel_x(marker: dict[str, Any]) -> float:
    pixel = marker.get("pixel", {})
    return float(pixel.get("x", 0.0))


def _pixel_y(marker: dict[str, Any]) -> float:
    pixel = marker.get("pixel", {})
    return float(pixel.get("y", 0.0))
