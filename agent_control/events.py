"""Append-only, secret-free event logging for agent evaluation."""

from __future__ import annotations

import json
import os
import time
import uuid
from pathlib import Path
from typing import Any


class EventLogger:
    def __init__(self, path: str | None = None) -> None:
        self.path = Path(path or os.environ.get("XUEGEROS_AGENT_EVENT_LOG", "")) if (path or os.environ.get("XUEGEROS_AGENT_EVENT_LOG")) else None

    def append(self, kind: str, payload: dict[str, Any], *, session_id: str = "", turn_id: str = "") -> str:
        event_id = uuid.uuid4().hex
        record = {
            "event_id": event_id,
            "observed_at": time.time(),
            "kind": kind,
            "session_id": str(session_id)[:120],
            "turn_id": str(turn_id)[:120],
            "payload": payload,
        }
        if self.path is not None:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with self.path.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n")
        return event_id
