#!/usr/bin/env python3
"""Shared helpers for ECBS dummy test nodes (log + msg build)."""

from __future__ import annotations

import json
import os
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, List, Optional

from ecbs_planner.msg import EvaluatePathWaypoint, FromZoneManager, WaitDependency


def default_log_dir() -> Path:
    # Prefer workspace-local logs when run from FMS_ECBS; else /tmp.
    candidates = [
        Path.cwd() / "ecbs_test_logs",
        Path.home() / "FMS_ECBS" / "ecbs_test_logs",
        Path("/tmp/ecbs_test_logs"),
    ]
    for c in candidates:
        try:
            c.mkdir(parents=True, exist_ok=True)
            return c
        except OSError:
            continue
    p = Path("/tmp/ecbs_test_logs")
    p.mkdir(parents=True, exist_ok=True)
    return p


def stamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")


class TextLogger:
    """Append human-readable + JSON blocks to one text file."""

    def __init__(self, path: Path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with self.path.open("a", encoding="utf-8") as f:
            f.write(f"\n{'=' * 72}\n")
            f.write(f"# session start {stamp()} pid={os.getpid()}\n")
            f.write(f"{'=' * 72}\n")

    def write(self, title: str, payload: Any) -> None:
        block = {
            "time": stamp(),
            "title": title,
            "data": payload,
        }
        text = json.dumps(block, indent=2, default=str)
        with self.path.open("a", encoding="utf-8") as f:
            f.write(f"\n--- {title} @ {block['time']} ---\n")
            f.write(text)
            f.write("\n")


def make_wp(
    t_sec: float,
    x: float,
    y: float,
    yaw: float = 0.0,
    level_name: str = "L1",
    index: int = -1,
) -> EvaluatePathWaypoint:
    wp = EvaluatePathWaypoint()
    wp.t_sec = float(t_sec)
    wp.x = float(x)
    wp.y = float(y)
    wp.yaw = float(yaw)
    wp.level_name = str(level_name)
    wp.index = int(index)
    # wait_sec / waiting_for left at defaults — evaluate omits; register fills when needed
    return wp


def make_from_zone(
    amr_id: str,
    fleet: str,
    task: str,
    revision: int,
    waypoints: List[EvaluatePathWaypoint],
    phase_index: int = 0,
) -> FromZoneManager:
    msg = FromZoneManager()
    msg.amr_id = amr_id
    msg.fleet_name = fleet
    msg.task_id = task
    msg.phase_index = int(phase_index)
    msg.path_revision = int(revision)
    msg.waypoints = list(waypoints)
    return msg


def wp_to_dict(wp: EvaluatePathWaypoint, *, include_waits: bool = True) -> dict:
    """Serialize waypoint. On evaluate logs, set include_waits=False to omit empty wait fields."""
    item = {
        "t_sec": wp.t_sec,
        "x": wp.x,
        "y": wp.y,
        "yaw": wp.yaw,
        "level_name": wp.level_name,
        "index": wp.index,
    }
    if not include_waits:
        return item
    has_wait = float(wp.wait_sec) > 1e-9 or len(wp.waiting_for) > 0
    if not has_wait:
        return item
    item["wait_sec"] = wp.wait_sec
    item["waiting_for"] = [
        {
            "amr_id": dep.amr_id,
            "wait_point": {
                "x": getattr(dep.wait_point, "x", 0.0),
                "y": getattr(dep.wait_point, "y", 0.0),
                "index": getattr(dep.wait_point, "index", -1),
                "wait_sec": getattr(dep.wait_point, "wait_sec", 0.0),
            },
            "clear_points": [{"x": c.x, "y": c.y} for c in dep.clear_points],
        }
        for dep in wp.waiting_for
    ]
    return item


def from_zone_to_dict(msg: FromZoneManager) -> dict:
    return {
        "amr_id": msg.amr_id,
        "fleet_name": msg.fleet_name,
        "task_id": msg.task_id,
        "phase_index": msg.phase_index,
        "path_revision": msg.path_revision,
        "waypoints": [wp_to_dict(w) for w in msg.waypoints],
    }


def request_to_dict(req) -> dict:
    # evaluate: geometry only in logs; register: include wait fields when set
    mode = str(getattr(req, "mode", "")).strip().lower()
    include_waits = mode != "evaluate"
    return {
        "mode": req.mode,
        "amr_id": req.amr_id,
        "fleet_name": req.fleet_name,
        "task_id": req.task_id,
        "phase_index": req.phase_index,
        "path_revision": req.path_revision,
        "query_horizon_m": req.query_horizon_m,
        "current_x": req.current_x,
        "current_y": req.current_y,
        "waypoints": [
            wp_to_dict(w, include_waits=include_waits) for w in req.waypoints
        ],
    }


def response_to_dict(res) -> dict:
    waiting_for = []
    if hasattr(res, "waiting_for"):
        for dep in res.waiting_for:
            item = {
                "amr_id": dep.amr_id,
                "wait_point": {
                    "x": dep.wait_point.x,
                    "y": dep.wait_point.y,
                    "index": dep.wait_point.index,
                    "wait_sec": dep.wait_point.wait_sec,
                },
                "clear_points": [
                    {"x": c.x, "y": c.y} for c in dep.clear_points
                ],
            }
            waiting_for.append(item)
    return {
        "success": res.success,
        "message": res.message,
        "amr_id": res.amr_id,
        "task_id": res.task_id,
        "path_revision": res.path_revision,
        "status": res.status,
        "horizon_m": res.horizon_m,
        "dep_from_index": getattr(res, "dep_from_index", -1),
        "dep_to_index": getattr(res, "dep_to_index", -1),
        "waiting_for": waiting_for,
    }


def build_corridor_east(
    start_x: float = 0.0,
    end_x: float = 100.0,
    step_m: float = 5.0,
    y: float = 0.0,
    speed_mps: float = 1.0,
    start_index: int = 0,
) -> List[EvaluatePathWaypoint]:
    """Path left→right. ETA t_sec = distance / speed."""
    wps: List[EvaluatePathWaypoint] = []
    x = start_x
    i = 0
    while x <= end_x + 1e-9:
        dist = abs(x - start_x)
        t = dist / max(speed_mps, 1e-6)
        wps.append(make_wp(t, x, y, 0.0, "L1", start_index + i))
        x += step_m
        i += 1
    return wps


def build_corridor_west(
    start_x: float = 100.0,
    end_x: float = 0.0,
    step_m: float = 5.0,
    y: float = 0.0,
    speed_mps: float = 1.0,
    start_index: int = 0,
    t0: float = 2.0,
) -> List[EvaluatePathWaypoint]:
    """Path right→left (head-on). Overlaps east corridor in space+time.

    Graph index must match east corridor at same x (ECBS same_cell uses index
    when both set). index = start_index + round(x / step_m).
    """
    wps: List[EvaluatePathWaypoint] = []
    x = start_x
    # Step toward end_x (decreasing).
    direction = -1.0 if end_x < start_x else 1.0
    while (direction < 0 and x >= end_x - 1e-9) or (
        direction > 0 and x <= end_x + 1e-9
    ):
        dist = abs(x - start_x)
        t = t0 + dist / max(speed_mps, 1e-6)
        yaw = 3.1415926535 if direction < 0 else 0.0
        idx = start_index + int(round(x / step_m))
        wps.append(make_wp(t, x, y, yaw, "L1", idx))
        x += direction * step_m
    return wps


def build_path1_amr1() -> List[EvaluatePathWaypoint]:
    """amr1 eastbound with ETA. Short enough for demo waits < max_wait."""
    # x: 0→50 m, 1 m/s, indices match x/5
    return build_corridor_east(
        start_x=0.0, end_x=50.0, step_m=5.0, y=0.0, speed_mps=1.0, start_index=0
    )


def build_conflict_amr2() -> List[EvaluatePathWaypoint]:
    """amr2 westbound overlapping amr1 in space+time.

    Same indices at same x so EcbsSolver::same_cell matches.
    t0=0 → head-on time overlap at shared cells (FIFO → amr2 waits).
    """
    return build_corridor_west(
        start_x=50.0,
        end_x=0.0,
        step_m=5.0,
        y=0.0,
        speed_mps=1.0,
        start_index=0,
        t0=0.0,
    )


def path_length_m(waypoints: List[EvaluatePathWaypoint]) -> float:
    if len(waypoints) < 2:
        return 0.0
    total = 0.0
    for i in range(1, len(waypoints)):
        a, b = waypoints[i - 1], waypoints[i]
        total += ((b.x - a.x) ** 2 + (b.y - a.y) ** 2) ** 0.5
    return total


def advance_pose_along_path(
    waypoints: Iterable[EvaluatePathWaypoint],
    travelled_m: float,
) -> tuple[float, float, float]:
    """Return (x, y, along_path_m) after travelling travelled_m from start."""
    wps = list(waypoints)
    if not wps:
        return 0.0, 0.0, 0.0
    if travelled_m <= 0.0:
        return wps[0].x, wps[0].y, 0.0

    along = 0.0
    for i in range(1, len(wps)):
        a, b = wps[i - 1], wps[i]
        seg = ((b.x - a.x) ** 2 + (b.y - a.y) ** 2) ** 0.5
        if along + seg >= travelled_m:
            frac = (travelled_m - along) / seg if seg > 1e-9 else 0.0
            x = a.x + frac * (b.x - a.x)
            y = a.y + frac * (b.y - a.y)
            return x, y, travelled_m
        along += seg
    return wps[-1].x, wps[-1].y, along
