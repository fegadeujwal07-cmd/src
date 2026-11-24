#!/usr/bin/env python3
"""Dummy node #2: EvaluatePath client.

Every --period seconds (default 10):
  - call /evaluate_path_service mode=evaluate
  - query next --horizon-m metres (default 20)
  - advance simulated pose by horizon along stored path

Request + response logged to text file.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from ecbs_planner.srv import EvaluatePath

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ecbs_test_common as common  # noqa: E402


class DummyEvaluateClient(Node):
    def __init__(
        self,
        service_name: str,
        log_dir: Path,
        amr_id: str,
        task_id: str,
        revision: int,
        horizon_m: float,
        period_sec: float,
        path_dir: str,
    ):
        super().__init__("dummy_evaluate_client")
        self.cli = self.create_client(EvaluatePath, service_name)
        self.log = common.TextLogger(log_dir / "evaluate_req_res.txt")
        self.amr_id = amr_id
        self.task_id = task_id
        self.revision = revision
        self.horizon_m = horizon_m
        self.period_sec = period_sec
        self.travelled_m = 0.0
        self.tick = 0
        self._busy = False

        if path_dir == "west":
            self.path_wps = common.build_conflict_amr2()
        else:
            self.path_wps = common.build_path1_amr1()
        self.path_len_m = common.path_length_m(self.path_wps)
        self.get_logger().info(f"Waiting for {service_name} ...")
        if not self.cli.wait_for_service(timeout_sec=30.0):
            raise RuntimeError(f"Service {service_name} not available")

        self.get_logger().info(
            f"Evaluate every {period_sec}s horizon={horizon_m}m "
            f"amr={amr_id} path_dir={path_dir}; log={self.log.path}"
        )
        self.create_timer(1.0, self._first_then_period)
        self._started = False

    def _first_then_period(self):
        if self._started:
            return
        self._started = True
        self._kick_evaluate()
        self.create_timer(self.period_sec, self._kick_evaluate)

    def _kick_evaluate(self):
        if self._busy:
            self.get_logger().warn("Previous evaluate still pending; skip tick")
            return
        if not self.cli.service_is_ready():
            self.get_logger().warn("Service not ready")
            return

        x, y, _along = common.advance_pose_along_path(
            self.path_wps, self.travelled_m
        )
        req = EvaluatePath.Request()
        req.mode = "evaluate"
        req.amr_id = self.amr_id
        req.fleet_name = "amr_fleet"
        req.task_id = self.task_id
        req.phase_index = 0
        req.path_revision = self.revision
        req.waypoints = list(self.path_wps)
        req.query_horizon_m = float(self.horizon_m)
        req.current_x = float(x)
        req.current_y = float(y)

        self.tick += 1
        tick = self.tick
        travelled = self.travelled_m
        self.log.write(
            f"REQUEST evaluate tick={tick} travelled_m={travelled:.1f}",
            common.request_to_dict(req),
        )
        self.get_logger().info(
            f"[{tick}] evaluate amr={self.amr_id} pose=({x:.1f},{y:.1f}) "
            f"travelled={travelled:.0f}m horizon={self.horizon_m:.0f}m"
        )

        self._busy = True
        future = self.cli.call_async(req)

        def _done(fut):
            self._busy = False
            try:
                res = fut.result()
            except Exception as exc:  # noqa: BLE001
                self.log.write(
                    f"RESPONSE FAIL tick={tick}",
                    {"error": str(exc)},
                )
                self.get_logger().error(f"EvaluatePath failed: {exc}")
                return
            if res is None:
                self.log.write(
                    f"RESPONSE FAIL tick={tick}",
                    {"error": "None result"},
                )
                return

            self.log.write(
                f"RESPONSE evaluate tick={tick} status={res.status}",
                common.response_to_dict(res),
            )
            self.get_logger().info(
                f"[{tick}] → status={res.status} success={res.success} "
                f"horizon_m={res.horizon_m:.1f} "
                f"dep=[{getattr(res, 'dep_from_index', -1)},"
                f"{getattr(res, 'dep_to_index', -1)}] "
                f"waiting_for={len(getattr(res, 'waiting_for', []))} "
                f"msg={res.message}"
            )
            self.travelled_m = min(
                self.travelled_m + self.horizon_m, self.path_len_m
            )
            if self.travelled_m >= self.path_len_m - 1e-6:
                self.get_logger().info(
                    f"Reached end of corridor ({self.path_len_m:.0f} m)"
                )

        future.add_done_callback(_done)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--service", default="/evaluate_path_service")
    parser.add_argument("--log-dir", default="")
    parser.add_argument("--amr-id", default="amr1")
    parser.add_argument("--task-id", default="task_A")
    parser.add_argument("--revision", type=int, default=1)
    parser.add_argument("--horizon-m", type=float, default=20.0)
    parser.add_argument("--period", type=float, default=10.0)
    parser.add_argument(
        "--path-dir",
        choices=("east", "west"),
        default="east",
        help="east=path1 (amr1), west=conflict path (amr2)",
    )
    # Launch injects --ros-args; ignore unknown.
    args, _unknown = parser.parse_known_args(argv)

    log_dir = Path(args.log_dir) if args.log_dir else common.default_log_dir()

    rclpy.init(args=None)
    node = DummyEvaluateClient(
        service_name=args.service,
        log_dir=log_dir,
        amr_id=args.amr_id,
        task_id=args.task_id,
        revision=args.revision,
        horizon_m=args.horizon_m,
        period_sec=args.period,
        path_dir=args.path_dir,
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
    sys.exit(main())
