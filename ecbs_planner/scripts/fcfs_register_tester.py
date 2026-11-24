#!/usr/bin/env python3
"""Live FCFS registration gate for ECBS (Zone Manager stand-in).

Stays running. Watches the real fleet path:

  /from_zone_manager  → record who got their task first (accept order)
  /to_zone_manager    → see WAIT deps after evaluate
  /evaluate_path_service mode=register → later AMR waits, first AMR is GO

Rule: if A and B conflict, the AMR that was seeded first keeps GO; the later
one is registered as WAIT for the first (until it clears). Mutual wrong-way
waits on the first AMR are cleared by that register.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from ecbs_planner.msg import (
    FromZoneManager,
    RegisterUpdate,
    ToZoneManager,
    WaitDependency,
)
from ecbs_planner.srv import EvaluatePath

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ecbs_test_common as common  # noqa: E402


def _pair_key(a: str, b: str) -> str:
    return f"{a}|{b}" if a < b else f"{b}|{a}"


class FcfsRegisterNode(Node):
    def __init__(
        self,
        seed_topic: str,
        to_zone_topic: str,
        service_name: str,
        log_dir: Path,
        fleet: str,
        debounce_sec: float,
    ):
        super().__init__("fcfs_register_tester")
        qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.fleet = fleet
        self.debounce_sec = float(debounce_sec)
        self.log = common.TextLogger(log_dir / "fcfs_register.txt")
        self.cli = self.create_client(EvaluatePath, service_name)

        # amr_id → (task_id, path_revision, accept_seq)
        self.accepted: Dict[str, Tuple[str, int, int]] = {}
        self.accept_seq = 0
        # amr_id → list of WaitDependency from latest /to_zone_manager
        self.waits: Dict[str, List[WaitDependency]] = {}
        # pair_key → last register time
        self._last_reg: Dict[str, float] = {}
        # pair_key → (later task, first task) already registered
        self._done: Dict[str, Tuple[str, str]] = {}
        self._busy = False

        self.create_subscription(FromZoneManager, seed_topic, self._on_seed, qos)
        self.create_subscription(ToZoneManager, to_zone_topic, self._on_to_zone, qos)
        self.create_timer(0.5, self._pump)

        self.get_logger().info(
            f"FCFS gate live: seed={seed_topic} waits={to_zone_topic} "
            f"svc={service_name} log={self.log.path}"
        )

    def _on_seed(self, msg: FromZoneManager):
        if not msg.amr_id:
            return
        task = msg.task_id or ""
        prev = self.accepted.get(msg.amr_id)
        # New task (or first sight) advances FCFS order.
        if prev is None or (task and prev[0] != task):
            self.accept_seq += 1
            self.accepted[msg.amr_id] = (
                task,
                int(msg.path_revision),
                self.accept_seq,
            )
            self.log.write(
                f"ACCEPT seq={self.accept_seq} amr={msg.amr_id} task={task}",
                {"path_revision": msg.path_revision, "wps": len(msg.waypoints)},
            )
            self.get_logger().info(
                f"FCFS accept seq={self.accept_seq} amr={msg.amr_id} task={task}"
            )
        else:
            # Same task: refresh revision only.
            self.accepted[msg.amr_id] = (prev[0], int(msg.path_revision), prev[2])

    def _on_to_zone(self, msg: ToZoneManager):
        if not msg.amr_id:
            return
        deps: List[WaitDependency] = []
        for pt in msg.path:
            for dep in pt.waiting_for:
                if dep.amr_id and dep.amr_id != msg.amr_id:
                    deps.append(dep)
        # De-dupe by blocker id (keep first wait_point seen).
        uniq: Dict[str, WaitDependency] = {}
        for d in deps:
            if d.amr_id not in uniq:
                uniq[d.amr_id] = d
        self.waits[msg.amr_id] = list(uniq.values())
        if uniq:
            self.get_logger().info(
                f"Wait seen amr={msg.amr_id} for={list(uniq.keys())}"
            )
            self._try_register()

    def _pump(self):
        if not self._busy:
            self._try_register()

    def _first_later(self, a: str, b: str) -> Optional[Tuple[str, str]]:
        ia = self.accepted.get(a)
        ib = self.accepted.get(b)
        if ia is None or ib is None:
            return None
        if ia[2] < ib[2]:
            return a, b
        if ib[2] < ia[2]:
            return b, a
        # Same seq (shouldn't happen): lexical later replans / waits.
        return (a, b) if a < b else (b, a)

    def _try_register(self):
        if self._busy or not self.cli.service_is_ready():
            return
        # Collect unordered pairs that have any wait between them.
        pairs = set()
        for waiter, deps in self.waits.items():
            for d in deps:
                pairs.add(_pair_key(waiter, d.amr_id))
        now = time.time()
        for pk in pairs:
            a, b = pk.split("|", 1)
            order = self._first_later(a, b)
            if order is None:
                continue
            first, later = order
            last = self._last_reg.get(pk, 0.0)
            if now - last < self.debounce_sec:
                continue
            # Registered once for these tasks: ECBS keeps it until a new task.
            tasks = (self.accepted[later][0], self.accepted[first][0])
            if self._done.get(pk) == tasks:
                continue
            # Already correct: later waits for first, first does not wait for later.
            later_deps = self.waits.get(later, [])
            first_deps = self.waits.get(first, [])
            later_waits_for_first = any(d.amr_id == first for d in later_deps)
            first_waits_for_later = any(d.amr_id == later for d in first_deps)
            if later_waits_for_first and not first_waits_for_later:
                # Desired state already (or will be after a prior register).
                continue
            if not later_waits_for_first and not first_waits_for_later:
                continue

            dep = self._dep_for_later(later, first)
            if dep is None:
                self.get_logger().warn(
                    f"Skip register {later}→{first}: no wait_point available"
                )
                continue
            meta = self.accepted.get(later)
            if meta is None:
                continue
            task_id, revision, _ = meta
            self._busy = True
            self._last_reg[pk] = now
            self._send_register(later, first, dep, task_id, revision, pk)
            return  # one register per pump

    def _dep_for_later(self, later: str, first: str) -> Optional[WaitDependency]:
        for d in self.waits.get(later, []):
            if d.amr_id == first:
                return d
        # Flip: first was waiting for later — reuse that geometry for later→first.
        for d in self.waits.get(first, []):
            if d.amr_id == later:
                out = WaitDependency()
                out.amr_id = first
                out.wait_point = d.wait_point
                out.clear_points = list(d.clear_points)
                return out
        return None

    def _send_register(
        self,
        later: str,
        first: str,
        dep: WaitDependency,
        task_id: str,
        revision: int,
        pair_key: str,
    ):
        u = RegisterUpdate()
        u.amr_id = later
        u.task_id = task_id
        u.phase_index = 0
        u.path_revision = revision
        u.waiting_for = [dep]

        req = EvaluatePath.Request()
        req.mode = "register"
        req.fleet_name = self.fleet
        req.amr_id = later
        req.task_id = task_id
        req.path_revision = revision
        req.updates = [u]

        self.log.write(
            f"REQ REGISTER later={later} first={first} (FCFS)",
            {
                "later": later,
                "first": first,
                "task_id": task_id,
                "waiting_for": dep.amr_id,
                "wait_point": {
                    "x": dep.wait_point.x,
                    "y": dep.wait_point.y,
                    "index": dep.wait_point.index,
                },
            },
        )
        self.get_logger().warn(
            f"FCFS register: {later} WAIT for {first} (GO); "
            f"pair={pair_key}"
        )
        future = self.cli.call_async(req)

        def _done(fut):
            self._busy = False
            try:
                res = fut.result()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"register failed: {exc}")
                self.log.write("FAIL REGISTER", {"error": str(exc)})
                return
            if res is None:
                self.log.write("FAIL REGISTER", {"error": "empty"})
                return
            self.log.write("RES REGISTER", common.response_to_dict(res))
            if res.success:
                self._done[pair_key] = (
                    self.accepted[later][0],
                    self.accepted[first][0],
                )
            self.get_logger().info(
                f"register done later={later} status={res.status} "
                f"msg={res.message}"
            )
            # Drop first's wait locally so we don't immediately re-flip.
            self.waits[first] = [
                d for d in self.waits.get(first, []) if d.amr_id != later
            ]
            self.waits[later] = [dep]

        future.add_done_callback(_done)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed-topic", default="/from_zone_manager")
    parser.add_argument("--to-zone-topic", default="/to_zone_manager")
    parser.add_argument("--service", default="/evaluate_path_service")
    parser.add_argument("--log-dir", default="")
    parser.add_argument("--fleet", default="amr_fleet")
    parser.add_argument("--debounce-sec", type=float, default=3.0)
    args = parser.parse_args(argv)

    log_dir = Path(args.log_dir) if args.log_dir else common.default_log_dir()
    rclpy.init(args=None)
    node = FcfsRegisterNode(
        seed_topic=args.seed_topic,
        to_zone_topic=args.to_zone_topic,
        service_name=args.service,
        log_dir=log_dir,
        fleet=args.fleet,
        debounce_sec=args.debounce_sec,
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
