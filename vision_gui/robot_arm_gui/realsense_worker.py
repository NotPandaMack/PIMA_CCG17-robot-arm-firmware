from __future__ import annotations

import time
from typing import Any

from PySide6.QtCore import QThread, Signal

from .detection_worker import HSVProfile, detect_green_object, draw_detection_overlay, pixel_to_robot


class RealSenseWorker(QThread):
    frame_ready = Signal(object)
    detection_ready = Signal(object)
    camera_status = Signal(bool, str)

    def __init__(self, parent: Any = None) -> None:
        super().__init__(parent)
        self.profile = HSVProfile()
        self.homography: list[list[float]] | None = None
        self.detect_enabled = True
        self._running = False

    def configure(
        self,
        *,
        profile: HSVProfile | None = None,
        homography: list[list[float]] | None = None,
        detect_enabled: bool | None = None,
    ) -> None:
        if profile is not None:
            self.profile = profile
        if homography is not None:
            self.homography = homography
        if detect_enabled is not None:
            self.detect_enabled = detect_enabled

    def stop(self) -> None:
        self._running = False
        self.wait(1500)

    def run(self) -> None:
        try:
            import cv2
            import numpy as np
            import pyrealsense2 as rs
        except Exception as error:
            self.camera_status.emit(False, f"RealSense unavailable: {error}")
            return

        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        align = rs.align(rs.stream.color)
        self._running = True
        started = False
        try:
            profile = pipeline.start(config)
            started = True
            device = profile.get_device()
            depth_sensor = device.first_depth_sensor()
            depth_scale = float(depth_sensor.get_depth_scale())
            name = str(device.get_info(rs.camera_info.name))
            serial = str(device.get_info(rs.camera_info.serial_number))
            firmware = str(device.get_info(rs.camera_info.firmware_version))
            self.camera_status.emit(True, f"{name} {serial} running")
            while self._running:
                frames = pipeline.wait_for_frames(1000)
                aligned = align.process(frames)
                depth_frame = aligned.get_depth_frame()
                color_frame = aligned.get_color_frame()
                if not depth_frame or not color_frame:
                    self.camera_status.emit(False, "RealSense frame missing depth or color")
                    time.sleep(0.05)
                    continue

                color = np.asanyarray(color_frame.get_data())
                depth = np.asanyarray(depth_frame.get_data()).copy()
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_TURBO)
                intr = depth_frame.profile.as_video_stream_profile().intrinsics
                intrinsics = {
                    "width": int(intr.width),
                    "height": int(intr.height),
                    "fx": float(intr.fx),
                    "fy": float(intr.fy),
                    "ppx": float(intr.ppx),
                    "ppy": float(intr.ppy),
                    "model": str(intr.model),
                    "coeffs": [float(value) for value in intr.coeffs],
                }
                detection = detect_green_object(color, self.profile) if self.detect_enabled else None
                robot_xy = None
                if detection and detection.found and self.homography:
                    robot_xy = pixel_to_robot(detection.pixel_x or 0, detection.pixel_y or 0, self.homography)
                color_overlay = draw_detection_overlay(color, detection, robot_xy, calibrated=bool(self.homography)) if detection else color
                self.frame_ready.emit(
                    {
                        "color": color_overlay,
                        "rawColor": color,
                        "depth": depth,
                        "depthColormap": depth_colormap,
                        "depthScale": depth_scale,
                        "intrinsics": intrinsics,
                        "metadata": {"name": name, "serial": serial, "firmware": firmware},
                    }
                )
                if detection:
                    self.detection_ready.emit({"detection": detection, "robot_xy": robot_xy})
        except Exception as error:
            self.camera_status.emit(False, f"RealSense stopped: {error}")
        finally:
            if started:
                pipeline.stop()
            self._running = False
            self.camera_status.emit(False, "RealSense stopped")
