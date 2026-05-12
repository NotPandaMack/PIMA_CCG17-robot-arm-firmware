from __future__ import annotations

import json
from copy import deepcopy
from pathlib import Path
from typing import Any


GUI_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[2]
SETTINGS_PATH = GUI_ROOT / "user_settings.json"
LOCAL_CALIBRATION_PATH = REPO_ROOT / "config" / "vision_calibration.json"

DEFAULT_WORKSPACE = {
    "xMin": -180.0,
    "xMax": 180.0,
    "yMin": 60.0,
    "yMax": 285.0,
    "zMin": 0.0,
    "zMax": 220.0,
}

DEFAULT_HSV_PROFILE = {
    "lowerHue": 40,
    "upperHue": 85,
    "saturationMin": 80,
    "valueMin": 80,
    "minArea": 350.0,
}

DEFAULT_SETTINGS: dict[str, Any] = {
    "piUrl": "http://raspberrypi.local:8000",
    "websiteUrl": "http://raspberrypi.local:8000",
    "espUrl": "http://ESP8266_IP",
    "cameraIndex": 0,
    "sideCameraUrl": "",
    "sideCameraRtmpsPort": 1936,
    "sideCameraMjpegPort": 8090,
    "motionEnabled": False,
    "continuousSend": False,
    "sendRateHz": 5.0,
    "mockPi": False,
    "fakeEsp": False,
    "previewOnlyMode": True,
    "movementAdapterDryRun": True,
    "autoPickEnabled": False,
    "autoPickStableSec": 2.0,
    "autoPickCooldownSec": 8.0,
    "hsv": deepcopy(DEFAULT_HSV_PROFILE),
    "workspace": deepcopy(DEFAULT_WORKSPACE),
    "ghostPreviewPassed": False,
    "hoverOnlyMovementPassed": False,
    "firstHoverConfirmed": False,
    "firstPickupConfirmed": False,
    "autoPickConfirmed": False,
    "lastPage": "setup",
}


def load_settings(path: Path = SETTINGS_PATH) -> dict[str, Any]:
    settings = deepcopy(DEFAULT_SETTINGS)
    if path.exists():
        with path.open("r", encoding="utf-8") as file:
            raw = json.load(file)
        if isinstance(raw, dict):
            _deep_update(settings, raw)
    settings["piUrl"] = normalize_http_url(settings.get("piUrl", DEFAULT_SETTINGS["piUrl"]), with_port=True)
    settings["websiteUrl"] = normalize_http_url(settings.get("websiteUrl", settings["piUrl"]), with_port=True)
    settings["espUrl"] = normalize_http_url(settings.get("espUrl", DEFAULT_SETTINGS["espUrl"]), with_port=False)
    settings["cameraIndex"] = int(settings.get("cameraIndex", 0))
    settings["sideCameraUrl"] = str(settings.get("sideCameraUrl", "")).strip()
    settings["sideCameraRtmpsPort"] = int(settings.get("sideCameraRtmpsPort", 1936))
    settings["sideCameraMjpegPort"] = int(settings.get("sideCameraMjpegPort", 8090))
    return settings


def save_settings(settings: dict[str, Any], path: Path = SETTINGS_PATH) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as file:
        json.dump(settings, file, indent=2, sort_keys=True)
        file.write("\n")


def normalize_http_url(value: Any, with_port: bool) -> str:
    text = str(value or "").strip()
    if not text:
        return ""
    if not text.startswith(("http://", "https://")):
        text = f"http://{text}"
    text = text.rstrip("/")
    if with_port and ":" not in text.removeprefix("http://").removeprefix("https://"):
        text = f"{text}:8000"
    return text


def _deep_update(target: dict[str, Any], source: dict[str, Any]) -> None:
    for key, value in source.items():
        if isinstance(value, dict) and isinstance(target.get(key), dict):
            _deep_update(target[key], value)
        else:
            target[key] = value
