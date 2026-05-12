from __future__ import annotations

import socket
import subprocess
import time
from typing import Any

from PySide6.QtCore import QThread, Signal


class RtmpSideCameraRelay(QThread):
    relay_status = Signal(bool, str)
    ingest_url_ready = Signal(str)
    capture_url_ready = Signal(str)

    def __init__(
        self,
        *,
        host: str = "0.0.0.0",
        rtmp_port: int = 1936,
        mjpeg_port: int = 8090,
        app_name: str = "live",
        stream_key: str = "side",
        parent: Any = None,
    ) -> None:
        super().__init__(parent)
        self.host = host
        self.rtmp_port = int(rtmp_port)
        self.mjpeg_port = int(mjpeg_port)
        self.app_name = app_name.strip("/") or "live"
        self.stream_key = stream_key.strip("/") or "side"
        self._running = False
        self._process: subprocess.Popen[str] | None = None

    @property
    def ingest_url(self) -> str:
        return f"rtmp://{_local_lan_ip()}:{self.rtmp_port}/{self.app_name}/{self.stream_key}"

    @property
    def server_url(self) -> str:
        return f"rtmp://{_local_lan_ip()}:{self.rtmp_port}/{self.app_name}"

    @property
    def capture_url(self) -> str:
        return f"http://127.0.0.1:{self.mjpeg_port}/side.mjpg"

    def stop(self) -> None:
        self._running = False
        if self._process and self._process.poll() is None:
            self._process.terminate()
            try:
                self._process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self._process.kill()
        self.wait(1800)

    def run(self) -> None:
        self._running = True
        self.ingest_url_ready.emit(self.ingest_url)
        self.capture_url_ready.emit(self.capture_url)
        command = [
            "ffmpeg",
            "-hide_banner",
            "-loglevel",
            "warning",
            "-listen",
            "1",
            "-i",
            f"rtmp://{self.host}:{self.rtmp_port}/{self.app_name}/{self.stream_key}",
            "-an",
            "-vf",
            "fps=15",
            "-q:v",
            "5",
            "-f",
            "mpjpeg",
            "-listen",
            "1",
            self.capture_url,
        ]
        try:
            self._process = subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
        except FileNotFoundError:
            self.relay_status.emit(False, "FFmpeg is required for Camo RTMP ingest")
            return
        except Exception as error:
            self.relay_status.emit(False, f"RTMP relay failed to start: {error}")
            return

        self.relay_status.emit(True, f"Camo RTMP relay listening at {self.ingest_url}")
        recent_errors: list[str] = []
        while self._running and self._process.poll() is None:
            line = self._process.stderr.readline() if self._process.stderr else ""
            if line:
                text = line.strip()
                if text:
                    recent_errors.append(text)
                    recent_errors = recent_errors[-4:]
                if "Address already in use" in text:
                    self.relay_status.emit(False, text)
                elif "Connection" in text or "Stream" in text or "Opening" in text:
                    self.relay_status.emit(True, text)
            else:
                time.sleep(0.1)
        if self._running:
            code = self._process.poll()
            detail = " | ".join(recent_errors) if recent_errors else "no FFmpeg error output"
            self.relay_status.emit(False, f"RTMP relay stopped with code {code}: {detail}")


def _local_lan_ip() -> str:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("8.8.8.8", 80))
        return str(sock.getsockname()[0])
    except Exception:
        return socket.gethostbyname(socket.gethostname())
    finally:
        sock.close()
