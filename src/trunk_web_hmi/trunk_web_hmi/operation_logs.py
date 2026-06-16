from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path
from typing import List, Optional

from .models import OperationLog


def _log_dir() -> Path:
    return Path.home() / ".ros" / "trunk_web_hmi" / "logs"


def _log_path_for_now() -> Path:
    date_text = datetime.now().strftime("%Y%m%d")
    return _log_dir() / f"operation_{date_text}.jsonl"


def append_operation_log(log: OperationLog) -> None:
    path = _log_path_for_now()
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(log.dict(), ensure_ascii=False, sort_keys=True))
        stream.write("\n")


def read_operation_logs(limit: int = 200, level: Optional[str] = None) -> List[OperationLog]:
    limit = max(1, min(limit, 2000))
    logs: List[OperationLog] = []
    log_dir = _log_dir()
    if not log_dir.exists():
        return []

    for path in sorted(log_dir.glob("operation_*.jsonl"), reverse=True):
        with path.open("r", encoding="utf-8") as stream:
            lines = stream.readlines()
        for line in reversed(lines):
            if len(logs) >= limit:
                return list(reversed(logs))
            line = line.strip()
            if not line:
                continue
            try:
                item = OperationLog(**json.loads(line))
            except (json.JSONDecodeError, TypeError, ValueError):
                continue
            if level and item.level != level:
                continue
            logs.append(item)
    return list(reversed(logs))
