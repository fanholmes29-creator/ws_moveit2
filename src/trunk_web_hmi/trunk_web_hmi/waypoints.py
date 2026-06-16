from __future__ import annotations

import json
import threading
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import List

from .models import WaypointCreateRequest, WaypointRecord


class WaypointStore:
    def __init__(self, path: Path) -> None:
        self._path = path
        self._lock = threading.Lock()
        self._path.parent.mkdir(parents=True, exist_ok=True)

    def list_waypoints(self) -> List[WaypointRecord]:
        with self._lock:
            return self._load_locked()

    def create_waypoint(self, request: WaypointCreateRequest) -> WaypointRecord:
        with self._lock:
            waypoints = self._load_locked()
            record = WaypointRecord(
                id=str(uuid.uuid4()),
                name=request.name.strip(),
                kind=request.kind,
                note=request.note.strip(),
                joint_positions=request.joint_positions,
                target_pose=request.target_pose,
                created_at=datetime.now(timezone.utc).isoformat(),
            )
            waypoints.append(record)
            self._save_locked(waypoints)
            return record

    def delete_waypoint(self, waypoint_id: str) -> bool:
        with self._lock:
            waypoints = self._load_locked()
            remaining = [item for item in waypoints if item.id != waypoint_id]
            if len(remaining) == len(waypoints):
                return False
            self._save_locked(remaining)
            return True

    def _load_locked(self) -> List[WaypointRecord]:
        if not self._path.exists():
            return []
        with self._path.open("r", encoding="utf-8") as stream:
            raw = json.load(stream)
        return [WaypointRecord(**self._normalize_record(item)) for item in raw]

    def _save_locked(self, waypoints: List[WaypointRecord]) -> None:
        payload = [item.dict() for item in waypoints]
        tmp_path = self._path.with_suffix(".tmp")
        with tmp_path.open("w", encoding="utf-8") as stream:
            json.dump(payload, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
        tmp_path.replace(self._path)

    @staticmethod
    def _normalize_record(item: dict) -> dict:
        normalized = dict(item)
        # 第一版位姿点位使用 kind="pose"，读取时保留兼容；新建点位会使用
        # pose_quaternion / pose_rpy，以便前端能精确回填目标类型。
        if "kind" not in normalized and normalized.get("target_pose"):
            normalized["kind"] = "pose"
        return normalized
