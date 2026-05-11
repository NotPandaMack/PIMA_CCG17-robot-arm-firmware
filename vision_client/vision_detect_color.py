from __future__ import annotations

import argparse
import json
import os
import sys
import time
import urllib.error
import urllib.request
from datetime import UTC, datetime
from typing import Any

import cv2
import numpy as np


DEFAULT_PI_SERVER_URL = os.environ.get("PI_SERVER_URL", "http://raspberrypi.local:8000").rstrip("/")
OBJECT_NAME = "green_object"


def main() -> int:
    parser = argparse.ArgumentParser(description="Detect a bright green object and send targets to the Raspberry Pi.")
    parser.add_argument("--pi-url", default=DEFAULT_PI_SERVER_URL, help="Raspberry Pi vision server URL")
    parser.add_argument("--camera", type=int, default=0, help="OpenCV camera index")
    parser.add_argument("--max-hz", type=float, default=8.0, help="Maximum target POST rate")
    parser.add_argument("--min-area", type=float, default=350.0, help="Minimum contour area")
    parser.add_argument("--calibrate", action="store_true", help="Run guided calibration instead of detection")
    args = parser.parse_args()

    if args.calibrate:
        return run_guided_calibration(args.pi_url.rstrip("/"), args.camera)

    return run_detection(args.pi_url.rstrip("/"), args.camera, args.max_hz, args.min_area)


def run_detection(pi_url: str, camera_index: int, max_hz: float, min_area: float) -> int:
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f"Unable to open camera index {camera_index}", file=sys.stderr)
        return 1

    calibration = get_json(f"{pi_url}/vision/calibration") or {}
    homography = np.array(calibration.get("homography", []), dtype=np.float64)
    has_homography = homography.shape == (3, 3)
    post_interval = 1.0 / max(0.1, max_hz)
    last_post = 0.0
    last_error_at = 0.0

    print("Detection running.")
    print("q = quit, c = print HSV/calibration help")
    print(f"Pi server: {pi_url}")
    print(f"Calibration: {'loaded' if has_homography else 'not loaded; sending pixels only'}")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Camera read failed", file=sys.stderr)
            break

        detection = detect_green_object(frame, min_area)
        if detection:
            x, y, radius, confidence = detection
            cv2.circle(frame, (x, y), int(radius), (0, 255, 0), 2)
            cv2.circle(frame, (x, y), 4, (0, 0, 255), -1)
            cv2.putText(frame, f"{OBJECT_NAME} {confidence:.2f}", (x + 8, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            now = time.monotonic()
            if now - last_post >= post_interval:
                payload: dict[str, Any] = {
                    "type": "object_detected",
                    "object": OBJECT_NAME,
                    "pixelX": x,
                    "pixelY": y,
                    "confidence": confidence,
                }
                robot_xy = pixel_to_robot(x, y, homography) if has_homography else None
                if robot_xy is not None:
                    payload["robotX"], payload["robotY"] = robot_xy
                success = post_json(f"{pi_url}/vision/target", payload)
                if not success and now - last_error_at > 2.0:
                    print("Pi server offline or rejected target; continuing locally")
                    last_error_at = now
                last_post = now

        cv2.imshow("Robot Arm Vision - Green Object", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord("c"):
            print_hsv_help(has_homography)

    cap.release()
    cv2.destroyAllWindows()
    return 0


def detect_green_object(frame: np.ndarray, min_area: float) -> tuple[int, int, float, float] | None:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower = np.array([40, 80, 80])
    upper = np.array([85, 255, 255])
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(contour)
    if area < min_area:
        return None

    moments = cv2.moments(contour)
    if moments["m00"] == 0:
        return None

    x = int(moments["m10"] / moments["m00"])
    y = int(moments["m01"] / moments["m00"])
    _, radius = cv2.minEnclosingCircle(contour)
    frame_area = float(frame.shape[0] * frame.shape[1])
    confidence = max(0.0, min(0.99, 0.65 + min(0.34, area / frame_area * 15.0)))
    return x, y, radius, confidence


def run_guided_calibration(pi_url: str, camera_index: int) -> int:
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f"Unable to open camera index {camera_index}", file=sys.stderr)
        return 1

    ok, frame = cap.read()
    if not ok:
        print("Camera read failed", file=sys.stderr)
        return 1

    height, width = frame.shape[:2]
    print("\nGuided calibration")
    print("Mount the webcam so the full reachable table area, robot base, and markers are visible.")
    input("Press Enter when the camera is mounted and the table is clear.")

    origin = capture_click(cap, "Click robot/table origin, then press Enter", "origin")
    default_points = [
        ("front-left", -100.0, 120.0),
        ("front-right", 100.0, 120.0),
        ("back-left", -100.0, 240.0),
        ("back-right", 100.0, 240.0),
    ]

    points: list[dict[str, Any]] = []
    for label, default_x, default_y in default_points:
        print(f"\nPlace marker at {label}. Default robot coordinate: X={default_x}, Y={default_y} mm")
        robot_x = prompt_float("Robot X mm", default_x)
        robot_y = prompt_float("Robot Y mm", default_y)
        pixel = capture_click(cap, f"Click marker: {label}, then press Enter", label)
        validate_point_spacing(points, pixel)
        points.append(
            {
                "label": label,
                "pixel": {"x": pixel[0], "y": pixel[1]},
                "robot": {"x": robot_x, "y": robot_y},
            }
        )

    pixel_points = np.array([[p["pixel"]["x"], p["pixel"]["y"]] for p in points], dtype=np.float32)
    robot_points = np.array([[p["robot"]["x"], p["robot"]["y"]] for p in points], dtype=np.float32)
    homography, _ = cv2.findHomography(pixel_points, robot_points)
    if homography is None:
        print("Unable to compute homography. Check marker order and spacing.", file=sys.stderr)
        return 1

    reprojection_error = compute_reprojection_error(pixel_points, robot_points, homography)

    print("\nTable Z calibration")
    print("For each point, manually jog the arm until the claw/tip barely touches the table.")
    print("Do not let this script lower the arm. It only reads the ESP status after you press Enter.")
    table_points = []
    for label in ("center-near", "center-far", "left-mid", "right-mid"):
        input(f"Move to {label}, barely touch the table, then press Enter to save current ESP X/Y/Z.")
        status = get_json(f"{pi_url}/vision/esp/status") or {}
        table_points.append(
            {
                "label": label,
                "x": float(status.get("x", 0.0)),
                "y": float(status.get("y", 0.0)),
                "z": float(status.get("z", 0.0)),
            }
        )
        print(f"Saved {table_points[-1]}")

    print("\nPickup/table-skim pitch")
    print("Manually jog the arm to the desired safe pickup pitch near the table.")
    input("Press Enter to capture the current ESP pitch.")
    status = get_json(f"{pi_url}/vision/esp/status") or {}
    pickup_pitch = float(status.get("pitch", 0.0))

    calibration = {
        "status": "calibrated",
        "createdAt": datetime.now(UTC).isoformat().replace("+00:00", "Z"),
        "camera": {"width": width, "height": height},
        "origin": {"pixel": {"x": origin[0], "y": origin[1]}},
        "pickupPitchDeg": pickup_pitch,
        "homography": homography.tolist(),
        "points": points,
        "reprojectionError": reprojection_error,
        "tableZ": {"method": "plane", "points": table_points},
    }

    if not put_json(f"{pi_url}/vision/calibration", calibration):
        print("Failed to save calibration to Pi server", file=sys.stderr)
        return 1

    print("\nCalibration saved.")
    print(f"Reprojection error: {reprojection_error:.2f} mm")
    print(f"Pickup pitch: {pickup_pitch:.1f} deg")

    print("\nValidation test")
    pixel = capture_click(cap, "Click any table point for conversion validation", "validation")
    robot_xy = pixel_to_robot(pixel[0], pixel[1], homography)
    print(f"Pixel {pixel} converts to robot X={robot_xy[0]:.1f}, Y={robot_xy[1]:.1f}")
    if input("Run hover-only preview through the Pi service? Type yes: ").strip().lower() == "yes":
        post_json(
            f"{pi_url}/vision/target",
            {
                "type": "object_detected",
                "object": "calibration_validation",
                "pixelX": pixel[0],
                "pixelY": pixel[1],
                "robotX": robot_xy[0],
                "robotY": robot_xy[1],
                "confidence": 1.0,
            },
        )
        preview = post_json(f"{pi_url}/vision/pick/preview?hoverOnly=true", {})
        print("Hover-only preview requested. Check Pi service logs for generated commands.")
        if preview:
            print(json.dumps(preview, indent=2))

    cap.release()
    cv2.destroyAllWindows()
    print("\nCalibrated")
    return 0


def capture_click(cap: cv2.VideoCapture, instruction: str, label: str) -> tuple[int, int]:
    clicked: list[tuple[int, int]] = []

    def on_mouse(event: int, x: int, y: int, _flags: int, _param: Any) -> None:
        if event == cv2.EVENT_LBUTTONDOWN:
            clicked[:] = [(x, y)]

    window = f"Calibration - {label}"
    cv2.namedWindow(window)
    cv2.setMouseCallback(window, on_mouse)
    print(instruction)

    while True:
        ok, frame = cap.read()
        if not ok:
            raise RuntimeError("camera read failed")
        if clicked:
            cv2.circle(frame, clicked[0], 6, (0, 0, 255), -1)
            cv2.putText(frame, f"{clicked[0][0]}, {clicked[0][1]}", (clicked[0][0] + 8, clicked[0][1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame, instruction, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        cv2.imshow(window, frame)
        key = cv2.waitKey(20) & 0xFF
        if key in (13, 10) and clicked:
            cv2.destroyWindow(window)
            return clicked[0]
        if key == ord("q"):
            raise KeyboardInterrupt


def pixel_to_robot(pixel_x: int, pixel_y: int, homography: np.ndarray) -> tuple[float, float] | None:
    if homography.shape != (3, 3):
        return None
    point = np.array([[[float(pixel_x), float(pixel_y)]]], dtype=np.float32)
    transformed = cv2.perspectiveTransform(point, homography)
    return float(transformed[0][0][0]), float(transformed[0][0][1])


def compute_reprojection_error(pixel_points: np.ndarray, robot_points: np.ndarray, homography: np.ndarray) -> float:
    projected = cv2.perspectiveTransform(pixel_points.reshape(-1, 1, 2), homography).reshape(-1, 2)
    errors = np.linalg.norm(projected - robot_points, axis=1)
    return float(np.mean(errors))


def validate_point_spacing(points: list[dict[str, Any]], pixel: tuple[int, int]) -> None:
    for point in points:
        dx = point["pixel"]["x"] - pixel[0]
        dy = point["pixel"]["y"] - pixel[1]
        if (dx * dx + dy * dy) ** 0.5 < 25:
            raise ValueError("Calibration points are duplicated or too close together")


def prompt_float(label: str, default: float) -> float:
    value = input(f"{label} [{default}]: ").strip()
    if not value:
        return default
    return float(value)


def get_json(url: str) -> dict[str, Any] | None:
    try:
        with urllib.request.urlopen(url, timeout=2.5) as response:
            return json.loads(response.read().decode("utf-8"))
    except (urllib.error.URLError, TimeoutError, json.JSONDecodeError) as error:
        print(f"GET failed: {url}: {error}", file=sys.stderr)
        return None


def post_json(url: str, payload: dict[str, Any]) -> Any:
    data = json.dumps(payload).encode("utf-8")
    request = urllib.request.Request(url, data=data, headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(request, timeout=1.5) as response:
            body = response.read().decode("utf-8")
            return json.loads(body) if body else True
    except (urllib.error.URLError, TimeoutError, json.JSONDecodeError) as error:
        print(f"POST failed: {url}: {error}", file=sys.stderr)
        return False


def put_json(url: str, payload: dict[str, Any]) -> bool:
    data = json.dumps(payload).encode("utf-8")
    request = urllib.request.Request(url, data=data, headers={"Content-Type": "application/json"}, method="PUT")
    try:
        with urllib.request.urlopen(request, timeout=3.0) as response:
            return response.status < 400
    except (urllib.error.URLError, TimeoutError) as error:
        print(f"PUT failed: {url}: {error}", file=sys.stderr)
        return False


def print_hsv_help(has_homography: bool) -> None:
    print("\nHSV threshold is currently lower=[40,80,80], upper=[85,255,255].")
    print("Use bright green markers/objects and avoid green table backgrounds.")
    print(f"Calibration homography loaded: {has_homography}")
    print("Run calibration with: python vision_detect_color.py --calibrate --pi-url http://RASPBERRY_PI_IP:8000\n")


if __name__ == "__main__":
    raise SystemExit(main())

