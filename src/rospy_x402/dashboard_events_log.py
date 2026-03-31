"""
Mirror of KYR ~/.kyr/dashboard_events.jsonl schema for rospy_x402 (no catkin dep on KYR Python).
"""

from __future__ import annotations

import json
import os
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional


def _path() -> Path:
    base = Path(os.environ.get("KYR_HOME", Path.home() / ".kyr")).expanduser()
    return base / "dashboard_events.jsonl"


def append_dashboard_event(
    source: str,
    kind: str,
    summary: str,
    metadata: Optional[Dict[str, Any]] = None,
) -> None:
    try:
        p = _path()
        p.parent.mkdir(parents=True, exist_ok=True)
        line = {
            "timestamp_iso": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
            "source": source,
            "kind": kind,
            "summary": summary,
            "metadata": metadata or {},
        }
        with open(p, "a", encoding="utf-8") as f:
            f.write(json.dumps(line, ensure_ascii=False) + "\n")
    except OSError:
        pass
