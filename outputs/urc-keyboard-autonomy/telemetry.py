"""Append-only mission telemetry suitable for Pi post-run analysis."""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any


class JsonlTelemetry:
    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)

    def write(self, event: str, **fields: Any) -> None:
        record = {"time_ns": time.monotonic_ns(), "event": event, **fields}
        with self.path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, separators=(",", ":"), default=str) + "\n")
