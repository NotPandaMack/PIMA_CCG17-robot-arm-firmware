from __future__ import annotations

import logging
import threading
import time
from typing import Any

from fastapi import FastAPI, HTTPException, Query, Request
from fastapi.middleware.cors import CORSMiddleware

from .calibration import (
    DEFAULT_CALIBRATION_PATH,
    fit_direct_jog_table_z,
    load_calibration,
    reset_calibration,
    save_calibration,
)
from .config import config_to_dict, load_config, update_config
from .esp_client import EspClient
from .planner import build_pick_plan, execute_plan
from .store import TargetStore
from .validation import ValidationError, validate_target_payload, validate_website_vision_target_payload


logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
logger = logging.getLogger(__name__)

app = FastAPI(title="Robot Arm Vision Bridge")

# ESP status cache — limits ESP to at most 1 poll per second regardless of how many clients ask.
# The ESP8266 can only handle ~1-2 concurrent HTTP connections; without this, simultaneous polls
# from the WebUI, DesktopGUI auto-pick timer, and calibration tasks overwhelm it.
_esp_cache: dict[str, Any] = {}
_esp_cache_ts: float = 0.0
_esp_cache_lock = threading.Lock()
_ESP_CACHE_TTL = 1.0  # seconds


# Session state for jog-based Z calibration captures (not persisted until fit-z is called)
_z_jog_captures: list[dict[str, Any]] = []


def _get_cached_esp_status(base_url: str) -> dict[str, Any]:
    global _esp_cache, _esp_cache_ts
    now = time.monotonic()
    with _esp_cache_lock:
        if _esp_cache and now - _esp_cache_ts < _ESP_CACHE_TTL:
            return dict(_esp_cache)
        try:
            status = EspClient(base_url).get_status()
            _esp_cache = status
            _esp_cache_ts = now
            return dict(status)
        except Exception as error:
            if _esp_cache:
                return dict(_esp_cache) | {"stale": True, "error": str(error)}
            raise


store = TargetStore()
website_vision_store = TargetStore()
manual_control_store = TargetStore()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=False,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/health")
def health() -> dict[str, Any]:
    return {"ok": True}


@app.get("/vision/config")
def get_config() -> dict[str, Any]:
    return {"ok": True, "config": config_to_dict(load_config())}


@app.put("/vision/config")
async def put_config(request: Request) -> dict[str, Any]:
    payload = await request.json()
    try:
        config = update_config(payload)
    except ValueError as error:
        raise HTTPException(status_code=400, detail=str(error)) from error
    logger.warning("Updated vision server config")
    return {"ok": True, "config": config_to_dict(config)}


@app.post("/vision/target")
async def post_target(request: Request) -> dict[str, Any]:
    try:
        payload = await request.json()
        target = validate_target_payload(payload)
        stored = store.set(target)
    except ValidationError as error:
        raise HTTPException(status_code=400, detail=str(error)) from error
    except Exception as error:
        raise HTTPException(status_code=400, detail="invalid JSON payload") from error

    logger.info(
        "Vision target %s pixel=(%s,%s) robot=(%s,%s,%s) confidence=%.2f",
        stored.get("object"),
        stored.get("pixelX"),
        stored.get("pixelY"),
        stored.get("robotX"),
        stored.get("robotY"),
        stored.get("robotZ"),
        stored.get("confidence"),
    )
    return {"ok": True, "target": stored}


@app.get("/vision/target")
def get_target() -> dict[str, Any]:
    target = store.get()
    if target is None:
        return {"hasTarget": False, "target": None}
    return {"hasTarget": True, "target": target}


@app.post("/vision/clear")
def clear_target() -> dict[str, Any]:
    store.clear()
    logger.info("Vision target cleared")
    return {"ok": True, "hasTarget": False}


@app.post("/api/vision-target")
async def post_website_vision_target(request: Request) -> dict[str, Any]:
    try:
        payload = await request.json()
        target = validate_website_vision_target_payload(payload)
        stored = website_vision_store.set(target)
    except ValidationError as error:
        raise HTTPException(status_code=400, detail=str(error)) from error
    except Exception as error:
        raise HTTPException(status_code=400, detail="invalid JSON payload") from error

    logger.info(
        "Received vision target object=%s robot=(%.1f,%.1f,%.1f) pitch=%.1f confidence=%.2f",
        stored.get("object"),
        stored.get("robotX"),
        stored.get("robotY"),
        stored.get("robotZ"),
        stored.get("pitch"),
        stored.get("confidence"),
    )
    return {"ok": True, "target": stored}


@app.get("/api/vision-target")
def get_website_vision_target() -> dict[str, Any]:
    target = website_vision_store.get()
    if target is None:
        return {"hasTarget": False, "target": None}
    return {"hasTarget": True, "target": target}


@app.post("/api/vision-target/clear")
def clear_website_vision_target() -> dict[str, Any]:
    website_vision_store.clear()
    logger.info("Website vision target cleared")
    return {"ok": True, "hasTarget": False}


@app.post("/api/manual-control-state")
async def post_manual_control_state(request: Request) -> dict[str, Any]:
    try:
        payload = await request.json()
    except Exception as error:
        raise HTTPException(status_code=400, detail="invalid JSON payload") from error
    if not isinstance(payload, dict):
        raise HTTPException(status_code=400, detail="payload must be a JSON object")
    stored = manual_control_store.set(payload)
    logger.info("Received manual control state source=%s command=%s", stored.get("source"), stored.get("command"))
    return {"ok": True, "state": stored}


@app.get("/api/manual-control-state")
def get_manual_control_state() -> dict[str, Any]:
    state = manual_control_store.get()
    if state is None:
        return {"hasState": False, "state": None}
    return {"hasState": True, "state": state}


@app.post("/vision/pick/preview")
def preview_pick(hoverOnly: bool = Query(default=False)) -> dict[str, Any]:
    return build_pick_plan(store.get(), load_config(), load_calibration(), hoverOnly)


@app.post("/vision/pick")
def pick(hoverOnly: bool = Query(default=False)) -> dict[str, Any]:
    config = load_config()
    calibration = load_calibration()
    client = EspClient(config.esp_base_url)
    try:
        esp_status_val = _get_cached_esp_status(config.esp_base_url)
    except Exception as error:
        logger.warning("ESP unreachable during pick: %s", error)
        return {"ok": False, "sent": False, "errors": [f"ESP unreachable: {error}"], "commands": []}
    if esp_status_val.get("estop") is True:
        return {"ok": False, "sent": False, "errors": ["ESP ESTOP is active"], "commands": []}
    plan = build_pick_plan(store.get(), config, calibration, hoverOnly, current_position=esp_status_val)
    return execute_plan(plan, client, esp_status=esp_status_val)


@app.get("/vision/calibration")
def get_calibration() -> dict[str, Any]:
    return load_calibration()


@app.put("/vision/calibration")
async def put_calibration(request: Request) -> dict[str, Any]:
    payload = await request.json()
    if not isinstance(payload, dict):
        raise HTTPException(status_code=400, detail="calibration must be a JSON object")
    save_calibration(payload)
    logger.info("Saved vision calibration to %s", DEFAULT_CALIBRATION_PATH)
    return {"ok": True, "calibration": payload}


@app.post("/vision/calibration/reset")
def reset() -> dict[str, Any]:
    calibration = reset_calibration()
    logger.warning("Vision calibration reset")
    return {"ok": True, "calibration": calibration}


@app.get("/vision/esp/status")
def esp_status() -> dict[str, Any]:
    config = load_config()
    try:
        return _get_cached_esp_status(config.esp_base_url)
    except Exception as error:
        raise HTTPException(status_code=503, detail=f"ESP unreachable: {error}") from error


# --- Jog-based Z calibration ---
# The user jogs the arm to key positions using the WebUI IK controls, then clicks
# "Capture" buttons. No depth camera or image clicks required — the ESP reports
# the exact arm position, which is ground truth for Z calibration.

@app.post("/vision/calibration/z-capture")
async def z_capture(request: Request) -> dict[str, Any]:
    global _z_jog_captures
    payload = await request.json()
    role = payload.get("role")
    if role not in ("table", "hover"):
        raise HTTPException(status_code=400, detail="role must be 'table' or 'hover'")
    robot_y = payload.get("robotY")
    robot_z = payload.get("robotZ")
    if not isinstance(robot_y, (int, float)) or not isinstance(robot_z, (int, float)):
        raise HTTPException(status_code=400, detail="robotY and robotZ must be numbers")
    capture = {"role": role, "robotY": float(robot_y), "robotZ": float(robot_z)}
    _z_jog_captures.append(capture)
    logger.info("Z jog capture: role=%s Y=%.1f Z=%.1f", role, robot_y, robot_z)
    return {"ok": True, "captures": list(_z_jog_captures)}


@app.get("/vision/calibration/z-captures")
def get_z_captures() -> dict[str, Any]:
    return {"ok": True, "captures": list(_z_jog_captures)}


@app.post("/vision/calibration/z-captures/clear")
def clear_z_captures() -> dict[str, Any]:
    global _z_jog_captures
    _z_jog_captures = []
    return {"ok": True, "captures": []}


@app.post("/vision/calibration/fit-z")
async def fit_z(request: Request) -> dict[str, Any]:
    global _z_jog_captures
    payload = await request.json()
    hover_clearance_mm = float(payload.get("hoverClearanceMm", 60.0))
    if not _z_jog_captures:
        raise HTTPException(status_code=400, detail="no Z captures — jog the arm to key positions and capture them first")
    try:
        table_z_dict = fit_direct_jog_table_z(_z_jog_captures, hover_clearance_mm=hover_clearance_mm)
    except ValueError as error:
        raise HTTPException(status_code=400, detail=str(error)) from error
    calibration = load_calibration()
    calibration["tableZ"] = table_z_dict
    calibration["zAxisInverted"] = bool(table_z_dict["zAxisInverted"])
    if calibration.get("status") != "calibrated" and isinstance(calibration.get("homography"), list) and calibration["homography"]:
        calibration["status"] = "calibrated"
    save_calibration(calibration)
    _z_jog_captures = []
    logger.info("Saved direct-jog Z calibration: tableZ=%.1f zAxisInverted=%s", table_z_dict["z"], table_z_dict["zAxisInverted"])
    return {"ok": True, "tableZ": table_z_dict, "calibration": calibration}
