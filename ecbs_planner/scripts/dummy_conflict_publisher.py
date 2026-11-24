#!/usr/bin/env python3
"""Dummy node #3: publish CONFLICTING path on /from_zone_manager.

Waits until ECBS is subscribed, then publishes **once**.
Head-on vs path1 → FIFO waits on amr2 in ScheduleCache.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from ecbs_planner.msg import FromZoneManager

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ecbs_test_common as common  # noqa: E402


class DummyConflictPublisher(Node):
    def __init__(
        self,
        topic: str,
        log_dir: Path,
        amr_id: str,
        task_id: str,
        revision: int,
    ):
        super().__init__("dummy_conflict_publisher")
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

        self.waypoints = common.build_conflict_amr2()
        self.msg = common.make_from_zone(
            amr_id=amr_id,
            fleet="amr_fleet",
            task=task_id,
            revision=revision,
            waypoints=self.waypoints,
        )

        self._timer = self.create_timer(0.5, self._try_publish)
        self.get_logger().warn(
            f"Waiting for subscriber on {topic} before CONFLICT publish "
            f"amr={amr_id} wps={len(self.waypoints)}; log={self.log.path}"
        )

    def _try_publish(self):
        if self._done:
            return

        n_subs = self.pub.get_subscription_count()
        if n_subs < 1:
            self._wait_log_count += 1
            if self._wait_log_count == 1 or self._wait_log_count % 4 == 0:
                self.get_logger().info(
                    f"No subscriber on {self.topic} yet — waiting for ECBS "
                    f"(try={self._wait_log_count})"
                )
            return

        self.pub.publish(self.msg)
        payload = common.from_zone_to_dict(self.msg)
        self.log.write(
            f"PUBLISH /from_zone_manager CONFLICT ({self.amr_id}) ONCE",
            payload,
        )
        self.get_logger().warn(
            f"Published CONFLICT ONCE amr={self.amr_id} "
            f"wps={len(self.msg.waypoints)} "
            f"ETA_end={self.msg.waypoints[-1].t_sec:.1f}s "
            f"subs={n_subs} → {self.log.path}"
        )
        self.get_logger().warn(
            "What next: ECBS solve vs cached amr1 → waits on amr2 (FIFO). "
            "Point evaluate client at amr2 to see WAIT."
        )
        self._done = True
        self._timer.cancel()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/from_zone_manager")
    parser.add_argument("--log-dir", default="")
    parser.add_argument("--amr-id", default="amr2")
    parser.add_argument("--task-id", default="task_B")
    parser.add_argument("--revision", type=int, default=1)
    args, _unknown = parser.parse_known_args(argv)

    log_dir = Path(args.log_dir) if args.log_dir else common.default_log_dir()

    rclpy.init(args=None)
    node = DummyConflictPublisher(
        topic=args.topic,
        log_dir=log_dir,
        amr_id=args.amr_id,
        task_id=args.task_id,
        revision=args.revision,
    )
    try:
        # Spin until published once, then exit shortly after.
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < 60.0:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node._done and (time.time() - t0) > 1.0:
                break
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
