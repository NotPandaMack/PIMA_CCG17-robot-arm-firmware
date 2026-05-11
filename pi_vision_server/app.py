from __future__ import annotations

import logging
from typing import Any

from fastapi import FastAPI, HTTPException, Query, Request
from fastapi.middleware.cors import CORSMiddleware

from .calibration import (
    DEFAULT_CALIBRATION_PATH,
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
store = TargetStore()
website_vision_store = TargetStore()

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


@app.post("/vision/pick/preview")
def preview_pick(hoverOnly: bool = Query(default=False)) -> dict[str, Any]:
    return build_pick_plan(store.get(), load_config(), load_calibration(), hoverOnly)


@app.post("/vision/pick")
def pick(hoverOnly: bool = Query(default=False)) -> dict[str, Any]:
    config = load_config()
    calibration = load_calibration()
    plan = build_pick_plan(store.get(), config, calibration, hoverOnly)
    client = EspClient(config.esp_base_url)
    return execute_plan(plan, client)


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
    return EspClient(config.esp_base_url).get_status()
