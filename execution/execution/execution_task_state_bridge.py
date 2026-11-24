#!/usr/bin/env python3
"""
Task State Bridge Node

This node polls the RMF API Server REST API for task states and republishes
them to a ROS 2 topic. This allows fleet adapters to receive real-time task
status changes (queued → underway → completed).

Data Flow:
    RMF Core → API Server (DB) → This Bridge (Polling) → ROS Topic → Fleet Adapter
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String

import json
import threading
import time
from typing import Dict, Any
import requests

try:
    import jwt
    JWT_AVAILABLE = True
except ImportError:
    JWT_AVAILABLE = False


# Statuses after which a task will never change again. Only these are safe to
# forget when a task stops being returned by the API; anything else that goes
# missing is still running and must keep its cached state.
FINAL_STATUSES = frozenset({'completed', 'failed', 'canceled', 'cancelled', 'killed'})


class TaskStateBridge(Node):
    """
    Bridge node that polls RMF API Server REST API and republishes
    task state updates to a ROS 2 topic.

    Publishes to: /rmf_task_state_updates (String - JSON encoded task state)

    Task state JSON includes:
    - booking.id: task ID (e.g., "delivery.dispatch-0")
    - status: "queued", "underway", "completed", "failed", "canceled"
    - assigned_to.name: robot name
    - category: task type (e.g., "delivery")
    - detail: pickup/dropoff information
    """

    def __init__(self):
        super().__init__('task_state_bridge')

        # Declare parameters
        self.declare_parameter('api_server_url', 'http://localhost:8000')
        self.declare_parameter('poll_interval', 1.0)  # Poll every second
        self.declare_parameter('reconnect_interval', 5.0)
        self.declare_parameter('jwt_secret', 'rmfisawesome')  # Default JWT secret
        self.declare_parameter('jwt_aud', 'rmf_api_server')
        self.declare_parameter('jwt_iss', 'stub')

        # Get parameters
        self.api_server_url = self.get_parameter('api_server_url').value
        self.poll_interval = self.get_parameter('poll_interval').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value
        self.jwt_secret = self.get_parameter('jwt_secret').value
        self.jwt_aud = self.get_parameter('jwt_aud').value
        self.jwt_iss = self.get_parameter('jwt_iss').value

        # Remove trailing slash if present
        self.api_server_url = self.api_server_url.rstrip('/')

        # Generate JWT token for authentication
        self.auth_headers = self._generate_auth_headers()

        # Use TRANSIENT_LOCAL so new subscribers see current state
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=20  # Keep recent task states for late subscribers
        )

        self.task_state_pub = self.create_publisher(
            String,
            '/rmf_task_state_updates',
            qos
        )

        # Track task states to detect changes
        self.task_states: Dict[str, Dict[str, Any]] = {}
        # Preserve last non-null booking.priority by task_id because
        # some RMF updates omit or null this field in later state transitions.
        self.task_priority_cache: Dict[str, Any] = {}
        # Unfinished tasks we have already warned about going quiet, so the
        # warning is logged once per task rather than once per poll.
        self.quiet_tasks: set = set()
        self.should_run = True
        self.connected = False

        # Start polling thread
        self.poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self.poll_thread.start()

        self.get_logger().info(f"🌉 Task State Bridge started (API: {self.api_server_url})")

    def _generate_auth_headers(self) -> Dict[str, str]:
        """Generate JWT token for API authentication"""
        if not JWT_AVAILABLE:
            return {}
        try:
            payload = {
                'preferred_username': 'task_state_bridge',
                'aud': self.jwt_aud,
                'iss': self.jwt_iss,
                'exp': int(time.time()) + 86400 * 365,
            }
            token = jwt.encode(payload, self.jwt_secret, algorithm='HS256')
            return {'Authorization': f'Bearer {token}'}
        except Exception:
            return {}

    def _poll_loop(self):
        """Main polling loop - fetches task states from REST API"""
        while self.should_run:
            try:
                self._fetch_and_publish_tasks()
                self.connected = True
                time.sleep(self.poll_interval)
            except requests.exceptions.ConnectionError:
                if self.connected:
                    self.get_logger().warn("⚠️ Lost connection to API Server")
                    self.connected = False
                time.sleep(self.reconnect_interval)
            except Exception as e:
                self.get_logger().error(f"❌ Polling error: {e}")
                time.sleep(self.reconnect_interval)

    def _fetch_and_publish_tasks(self):
        """Fetch all task states from API and publish updates"""
        try:
            # 'standby' matters as much as the rest: RMF parks a task there
            # between being assigned and its first event going active. Leaving
            # it out of this filter made such a task invisible to the bridge —
            # it dropped out of the response, its cached state was discarded,
            # and no further update was ever published for it. The queued
            # VDA5050 order for that task then sat unreleased forever with no
            # error anywhere (patrol.dispatch-3, Aug 2026).
            url = (
                f"{self.api_server_url}/tasks"
                "?status=standby,queued,underway,completed,failed,canceled"
            )
            response = requests.get(url, headers=self.auth_headers, timeout=5)

            if response.status_code != 200:
                self.get_logger().warn(f"⚠️ API error: {response.status_code}")
                return

            tasks = response.json()

            # Track current task IDs from API response
            current_task_ids = set()

            for task_state in tasks:
                booking = task_state.get('booking', {})
                task_id = booking.get('id')
                if task_id:
                    current_task_ids.add(task_id)
                self._process_task_state(task_state)

            # Clean up tasks that are no longer in the API response (completed)
            self._cleanup_completed_tasks(current_task_ids)

        except requests.exceptions.Timeout:
            pass  # Silent timeout, will retry
        except json.JSONDecodeError:
            pass  # Silent JSON error, will retry

    def _cleanup_completed_tasks(self, current_task_ids: set):
        """Forget tasks that have finished and left the API response.

        Only tasks whose LAST SEEN status was final are dropped. A task that
        merely stops being returned while still running keeps its cached
        state, so that when it reappears we still compare against its real
        previous status instead of treating it as brand new.

        The previous version deleted every absent task regardless of status.
        That silently erased still-running tasks, which is how a task could
        vanish mid-execution and never produce another update.
        """
        for task_id in list(self.task_states.keys()):
            if task_id in current_task_ids:
                continue

            last_status = (self.task_states[task_id] or {}).get('status')
            if last_status in FINAL_STATUSES:
                del self.task_states[task_id]
                self.task_priority_cache.pop(task_id, None)
                continue

            # Still unfinished but no longer reported. Say so once, rather
            # than letting it disappear in silence.
            if task_id not in self.quiet_tasks:
                self.quiet_tasks.add(task_id)
                self.get_logger().warn(
                    f"🔇 Task {task_id} stopped appearing in the API response "
                    f"while still '{last_status}'. Keeping its cached state; "
                    f"anything waiting on its next status change is now stalled."
                )

    def _process_task_state(self, task_state: Dict[str, Any]):
        """Process a task state - publish on any status change"""
        try:
            booking = task_state.get('booking', {})
            task_id = booking.get('id', 'unknown')
            # Keep priority stable per task even if later updates send null/missing.
            incoming_priority = booking.get('priority')
            if incoming_priority is not None:
                self.task_priority_cache[task_id] = incoming_priority
            elif task_id in self.task_priority_cache:
                booking['priority'] = self.task_priority_cache[task_id]
            status = task_state.get('status', 'unknown')
            assigned_to = task_state.get('assigned_to', {})
            robot_name = assigned_to.get('name', 'unassigned') if assigned_to else 'unassigned'

            # It is being reported again, so retract the "gone quiet" warning
            # and let a future disappearance warn afresh.
            if task_id in self.quiet_tasks:
                self.quiet_tasks.discard(task_id)
                self.get_logger().info(
                    f"🔔 Task {task_id} is being reported again (status={status})"
                )

            prev_state = self.task_states.get(task_id)
            prev_status = prev_state.get('status') if prev_state else None

            # Publish when status changed. On first sight, seed the cache only for
            # terminal tasks still sitting in the API DB from a prior run — publishing
            # those on startup makes execution think patrol.dispatch-0 is already
            # completed and can ignore or mishandle a fresh dispatch with the same id.
            if prev_state is None and status in FINAL_STATUSES:
                self.task_states[task_id] = task_state
                return

            should_publish = (prev_state is None) or (prev_status != status)

            if should_publish:
                self.get_logger().info(f"📨 Task {task_id}: {prev_status} → {status} ({robot_name})")
                msg = String()
                msg.data = json.dumps(task_state)
                self.task_state_pub.publish(msg)

            self.task_states[task_id] = task_state

        except Exception as e:
            self.get_logger().error(f"❌ Error: {e}")

    def destroy_node(self):
        """Cleanup on shutdown"""
        self.should_run = False
        super().destroy_node()


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    try:
        bridge = TaskStateBridge()
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
