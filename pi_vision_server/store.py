from __future__ import annotations

from datetime import UTC, datetime
from threading import Lock
from typing import Any


class TargetStore:
    def __init__(self) -> None:
        self._lock = Lock()
        self._target: dict[str, Any] | None = None

    def set(self, target: dict[str, Any]) -> dict[str, Any]:
        stored = dict(target)
        stored["receivedAt"] = datetime.now(UTC).isoformat().replace("+00:00", "Z")
        with self._lock:
            self._target = stored
        return stored

    def get(self) -> dict[str, Any] | None:
        with self._lock:
            if self._target is None:
                return None
            return dict(self._target)

    def clear(self) -> None:
        with self._lock:
            self._target = None

