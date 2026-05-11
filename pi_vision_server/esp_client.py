from __future__ import annotations

import json
import urllib.error
import urllib.parse
import urllib.request
from typing import Any


class EspClient:
    def __init__(self, base_url: str, timeout_sec: float = 2.0) -> None:
        self.base_url = base_url.rstrip("/")
        self.timeout_sec = timeout_sec

    def get_status(self) -> dict[str, Any]:
        with urllib.request.urlopen(f"{self.base_url}/status", timeout=self.timeout_sec) as response:
            return json.loads(response.read().decode("utf-8"))

    def send_command(self, command: str) -> None:
        query = urllib.parse.urlencode({"cmd": command})
        url = f"{self.base_url}/command?{query}"
        try:
            with urllib.request.urlopen(url, timeout=self.timeout_sec) as response:
                if response.status >= 400:
                    raise RuntimeError(f"ESP command failed with HTTP {response.status}: {command}")
        except urllib.error.URLError as error:
            raise RuntimeError(f"ESP command failed: {command}: {error}") from error

