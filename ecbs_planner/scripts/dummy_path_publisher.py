#!/usr/bin/env python3
"""Dummy node #1: publish full path (with ETA) on /from_zone_manager.

Waits until ECBS is subscribed, then publishes **once** (no republish).
Logs published path to text file for debug.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from ecbs_planner.msg import FromZoneManager

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ecbs_test_common as common  # noqa: E402


class DummyPathPublisher(Node):
    def __init__(
        self,
        topic: str,
        log_dir: Path,
        amr_id: str,
        task_id: str,
        revision: int,
    ):
        super().__init__("dummy_path_publisher")
        qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub = self.create_publisher(FromZoneManager, topic, qos)
        self.log = common.TextLogger(log_dir / "published_paths.txt")
        self.amr_id = amr_id
        self.topic = topic
        self._done = False
        self._wait_log_count = 0

        self.waypoints = common.build_path1_amr1()
        self.msg = common.make_from_zone(
            amr_id=amr_id,
            fleet="amr_fleet",
            task=task_id,
            revision=revision,
            waypoints=self.waypoints,
        )

        # Poll until ECBS subscribes, then publish once.
        self._timer = self.create_timer(0.5, self._try_publish)
        self.get_logger().info(
            f"Waiting for subscriber on {topic} before publishing "
            f"amr={amr_id} wps={len(self.waypoints)}; log={self.log.path}"
        )

    def _try_publish(self):
        if self._done:
            return

        n_subs = self.pub.get_subscription_count()
        if n_subs < 1:
            self._wait_log_count += 1
            # Log every ~2s (every 4th tick) so terminal not spammy.
            if self._wait_log_count == 1 or self._wait_log_count % 4 == 0:
                self.get_logger().info(
                    f"No subscriber on {self.topic} yet — waiting for ECBS "
                    f"(try={self._wait_log_count})"
                )
            return

        self.pub.publish(self.msg)
        payload = common.from_zone_to_dict(self.msg)
        self.log.write("PUBLISH /from_zone_manager (path1) ONCE", payload)
        self.get_logger().info(
            f"Published ONCE path1 amr={self.amr_id} "
            f"wps={len(self.msg.waypoints)} "
            f"ETA_end={self.msg.waypoints[-1].t_sec:.1f}s "
            f"subs={n_subs} → {self.log.path}"
        )
        self._done = True
        # Stop timer — no republish.
        self._timer.cancel()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/from_zone_manager")
    parser.add_argument("--log-dir", default="")
    parser.add_argument("--amr-id", default="amr1")
    parser.add_argument("--task-id", default="task_A")
    parser.add_argument("--revision", type=int, default=1)
    # Launch injects --ros-args; ignore unknown.
    args, _unknown = parser.parse_known_args(argv)

    log_dir = Path(args.log_dir) if args.log_dir else common.default_log_dir()

    rclpy.init(args=None)
    node = DummyPathPublisher(
        topic=args.topic,
        log_dir=log_dir,
        amr_id=args.amr_id,
        task_id=args.task_id,
        revision=args.revision,
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
