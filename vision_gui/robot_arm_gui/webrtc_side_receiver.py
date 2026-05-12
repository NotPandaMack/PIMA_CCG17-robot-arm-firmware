from __future__ import annotations

import asyncio
import socket
from typing import Any

from PySide6.QtCore import QThread, Signal


class WhipSideCameraReceiver(QThread):
    frame_ready = Signal(object)
    camera_status = Signal(bool, str)
    ingest_url_ready = Signal(str)

    def __init__(self, *, host: str = "0.0.0.0", port: int = 8899, path: str = "/whip/side", parent: Any = None) -> None:
        super().__init__(parent)
        self.host = host
        self.port = int(port)
        self.path = path
        self._loop: asyncio.AbstractEventLoop | None = None
        self._stop_event: asyncio.Event | None = None
        self._peers: set[Any] = set()

    @property
    def ingest_url(self) -> str:
        return f"http://{_local_lan_ip()}:{self.port}{self.path}"

    def stop(self) -> None:
        if self._loop and self._stop_event:
            self._loop.call_soon_threadsafe(self._stop_event.set)
        self.wait(1800)

    def run(self) -> None:
        try:
            asyncio.run(self._run())
        except Exception as error:
            self.camera_status.emit(False, f"WebRTC side receiver failed: {error}")

    async def _run(self) -> None:
        try:
            from aiohttp import web
            from aiortc import RTCPeerConnection, RTCSessionDescription
        except Exception as error:
            self.camera_status.emit(False, f"Install GUI WebRTC requirements: {error}")
            return

        self._loop = asyncio.get_running_loop()
        self._stop_event = asyncio.Event()

        async def handle_whip(request: Any) -> Any:
            offer_sdp = await request.text()
            peer = RTCPeerConnection()
            self._peers.add(peer)

            @peer.on("connectionstatechange")
            async def on_connectionstatechange() -> None:
                state = peer.connectionState
                if state == "connected":
                    self.camera_status.emit(True, "Larix WebRTC stream connected")
                elif state in {"failed", "closed", "disconnected"}:
                    self.camera_status.emit(False, f"Larix WebRTC stream {state}")
                    await peer.close()
                    self._peers.discard(peer)

            @peer.on("track")
            def on_track(track: Any) -> None:
                if track.kind != "video":
                    return
                self.camera_status.emit(True, "Larix WebRTC video receiving")
                asyncio.create_task(self._receive_video(track))

            await peer.setRemoteDescription(RTCSessionDescription(sdp=offer_sdp, type="offer"))
            answer = await peer.createAnswer()
            await peer.setLocalDescription(answer)
            return web.Response(
                status=201,
                text=peer.localDescription.sdp,
                content_type="application/sdp",
                headers={"Location": self.path},
            )

        async def handle_options(_request: Any) -> Any:
            return web.Response(status=204)

        app = web.Application()
        app.router.add_route("OPTIONS", self.path, handle_options)
        app.router.add_post(self.path, handle_whip)
        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, self.host, self.port)
        await site.start()

        self.ingest_url_ready.emit(self.ingest_url)
        self.camera_status.emit(True, f"WebRTC WHIP receiver listening at {self.ingest_url}")
        try:
            await self._stop_event.wait()
        finally:
            await asyncio.gather(*(peer.close() for peer in list(self._peers)), return_exceptions=True)
            self._peers.clear()
            await runner.cleanup()
            self.camera_status.emit(False, "WebRTC WHIP receiver stopped")

    async def _receive_video(self, track: Any) -> None:
        while True:
            try:
                frame = await track.recv()
            except Exception:
                return
            self.frame_ready.emit(frame.to_ndarray(format="bgr24"))


def _local_lan_ip() -> str:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("8.8.8.8", 80))
        return str(sock.getsockname()[0])
    except Exception:
        return socket.gethostbyname(socket.gethostname())
    finally:
        sock.close()
