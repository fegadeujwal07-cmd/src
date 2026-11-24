#!/usr/bin/env python3
"""Mock zone_manager:

  1) Publish full paths on /from_zone_manager  (ECBS registers into ScheduleCache)
  2) Call /evaluate_path_service mode=evaluate for next ~20 m FREE/WAIT
"""

from __future__ import annotations

import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node

from ecbs_planner.msg import EvaluatePathWaypoint, FromZoneManager
from ecbs_planner.srv import EvaluatePath


def make_wp(t_sec, x, y, yaw=0.0, level_name="L1", index=-1):
    wp = EvaluatePathWaypoint()
    wp.t_sec = float(t_sec)
    wp.x = float(x)
    wp.y = float(y)
    wp.yaw = float(yaw)
    wp.level_name = str(level_name)
    wp.index = int(index)
    # evaluate: omit wait_sec / waiting_for (ROS defaults 0 / [])
    return wp


def response_to_dict(res: EvaluatePath.Response) -> dict:
    return {
        "success": res.success,
        "message": res.message,
        "amr_id": res.amr_id,
        "task_id": res.task_id,
        "path_revision": res.path_revision,
        "status": res.status,
        "horizon_m": res.horizon_m,
        "dep_from_index": res.dep_from_index,
        "dep_to_index": res.dep_to_index,
        "waiting_for": [
            {
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
            for dep in res.waiting_for
        ],
    }


def make_from_zone(amr_id, fleet, task, revision, waypoints) -> FromZoneManager:
    msg = FromZoneManager()
    msg.amr_id = amr_id
    msg.fleet_name = fleet
    msg.task_id = task
    msg.phase_index = 0
    msg.path_revision = revision
    msg.waypoints = waypoints
    return msg


class MockZoneManager(Node):
    def __init__(self, service_name: str, from_topic: str):
        super().__init__("mock_zone_manager_client")
        self.pub = self.create_publisher(FromZoneManager, from_topic, 10)
        self.cli = self.create_client(EvaluatePath, service_name)
        self.get_logger().info(f"Waiting for {service_name} ...")
        if not self.cli.wait_for_service(timeout_sec=15.0):
            raise RuntimeError(f"Service {service_name} not available")
        # Give subscription time to connect.
        time.sleep(0.5)

    def publish_path(self, msg: FromZoneManager):
        self.pub.publish(msg)
        self.get_logger().info(
            f"Published /from_zone_manager amr={msg.amr_id} wps={len(msg.waypoints)}"
        )
        time.sleep(0.3)  # allow ECBS to process

    def evaluate(self, amr_id, task, revision, x, y, waypoints, horizon_m=20.0):
        req = EvaluatePath.Request()
        req.mode = "evaluate"
        req.amr_id = amr_id
        req.fleet_name = "amr_fleet"
        req.task_id = task
        req.phase_index = 0
        req.path_revision = revision
        req.waypoints = list(waypoints)
        req.query_horizon_m = float(horizon_m)
        req.current_x = float(x)
        req.current_y = float(y)
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done() or future.result() is None:
            raise RuntimeError("evaluate call failed")
        return future.result()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--service", default="/evaluate_path_service")
    parser.add_argument("--from-topic", default="/from_zone_manager")
    args = parser.parse_args(argv)

    rclpy.init(args=None)
    node = MockZoneManager(args.service, args.from_topic)

    try:
        print("\n=== Publish amr1 full path on /from_zone_manager ===")
        print(json.dumps({
            "amr_id": "amr1",
            "fleet_name": "amr_fleet",
            "task_id": "task_A",
            "phase_index": 0,
            "path_revision": 1,
            "waypoints": [
                {"t_sec": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "level_name": "L1", "index": 1},
                {"t_sec": 5.0, "x": 5.0, "y": 0.0, "yaw": 0.0, "level_name": "L1", "index": 5},
                {"t_sec": 10.0, "x": 10.0, "y": 0.0, "yaw": 0.0, "level_name": "L1", "index": 10},
            ],
        }, indent=2))
        node.publish_path(make_from_zone(
            "amr1", "amr_fleet", "task_A", 1,
            [
                make_wp(0.0, 0.0, 0.0, 0.0, "L1", 1),
                make_wp(5.0, 5.0, 0.0, 0.0, "L1", 5),
                make_wp(10.0, 10.0, 0.0, 0.0, "L1", 10),
            ],
        ))

        print("\n=== Publish amr2 overlapping path on /from_zone_manager ===")
        node.publish_path(make_from_zone(
            "amr2", "amr_fleet", "task_B", 1,
            [
                make_wp(2.0, 12.0, 0.0, 3.14, "L1", 12),
                make_wp(6.0, 8.0, 0.0, 3.14, "L1", 8),
                make_wp(9.0, 5.0, 0.0, 3.14, "L1", 5),
                make_wp(12.0, 0.0, 0.0, 3.14, "L1", 1),
            ],
        ))

        print("\n=== Evaluate amr2 next 20 m (service) ===")
        amr2_wps = [
            make_wp(2.0, 12.0, 0.0, 3.14, "L1", 12),
            make_wp(6.0, 8.0, 0.0, 3.14, "L1", 8),
            make_wp(9.0, 5.0, 0.0, 3.14, "L1", 5),
            make_wp(12.0, 0.0, 0.0, 3.14, "L1", 1),
        ]
        res = node.evaluate("amr2", "task_B", 1, 12.0, 0.0, amr2_wps, 20.0)
        print(json.dumps(response_to_dict(res), indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
