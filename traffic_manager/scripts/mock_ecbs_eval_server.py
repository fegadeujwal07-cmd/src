#!/usr/bin/env python3
# Copyright 2026. Test ECBS EvaluatePath server + path release markers (yellow / blue).

from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from ecbs_planner.msg import ClearPoint, WaitDependency, WaitPoint
from ecbs_planner.srv import EvaluatePath
from traffic_manager.msg import PathIntent, PathRelease
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


def _waypoint_path_index(wp_index: int, slice_index: int) -> int:
    return int(wp_index) if wp_index >= 0 else slice_index


class MockEcbsEvalServer(Node):
    """Serves /evaluate_path_service for integration tests with traffic_manager."""

    def __init__(self) -> None:
        super().__init__('mock_ecbs_eval_server')

        self.declare_parameter('evaluate_path_service', '/evaluate_path_service')
        self.declare_parameter('path_intent_topic', '/traffic_manager/path_intent')
        self.declare_parameter('path_release_topic', '/traffic_manager/path_release')
        self.declare_parameter('marker_topic', '/traffic_manager/path_release_markers')
        self.declare_parameter('blocking_amr_id', 'amr2')
        # RViz needs a TF frame (e.g. map). RMF level_name is not always a valid frame.
        self.declare_parameter('marker_frame_id', 'map')
        self.declare_parameter('use_level_name_as_marker_frame', False)
        self.declare_parameter('marker_republish_hz', 2.0)

        svc_name = self.get_parameter('evaluate_path_service').value
        intent_topic = self.get_parameter('path_intent_topic').value
        release_topic = self.get_parameter('path_release_topic').value
        marker_topic = self.get_parameter('marker_topic').value

        self._blocking_amr = self.get_parameter('blocking_amr_id').value
        self._marker_frame = self.get_parameter('marker_frame_id').value
        self._use_level_frame = self.get_parameter('use_level_name_as_marker_frame').value

        self._paths: Dict[str, List] = {}
        self._release_through: Dict[str, int] = {}
        # Leg key -> global path index of last WAIT wait_point (cleared after robot passes it).
        self._wait_hold_index: Dict[Tuple[str, str, int], int] = {}
        self._last_marker_array: Optional[MarkerArray] = None

        # RViz Marker display uses TRANSIENT_LOCAL; VOLATILE publishers are incompatible.
        marker_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._marker_pub = self.create_publisher(MarkerArray, marker_topic, marker_qos)
        self.create_subscription(PathIntent, intent_topic, self._on_path_intent, 10)
        self.create_subscription(PathRelease, release_topic, self._on_path_release, 10)

        republish_hz = float(self.get_parameter('marker_republish_hz').value)
        if republish_hz > 0.0:
            self.create_timer(1.0 / republish_hz, self._republish_markers)

        self._srv = self.create_service(EvaluatePath, svc_name, self._handle_evaluate)
        self.get_logger().info(
            f'Mock ECBS listening on [{svc_name}], markers on [{marker_topic}] '
            f'(QoS: TRANSIENT_LOCAL for RViz)'
        )

    def _republish_markers(self) -> None:
        if self._last_marker_array is not None:
            self._marker_pub.publish(self._last_marker_array)

    def _on_path_intent(self, msg: PathIntent) -> None:
        if not msg.robot_name:
            return
        self._paths[msg.robot_name] = list(msg.path)
        key = (msg.robot_name, msg.task_id, msg.path_revision)
        self._wait_hold_index.pop(key, None)
        self._release_through.pop(msg.robot_name, None)

    def _on_path_release(self, msg: PathRelease) -> None:
        if not msg.robot_name or not msg.approved:
            return
        if msg.release_through_index < 0:
            return

        path = self._paths.get(msg.robot_name)
        if not path:
            return

        self._release_through[msg.robot_name] = msg.release_through_index
        full_last = len(path) - 1
        partial = msg.release_through_index < full_last

        if partial:
            wp = path[msg.release_through_index]
            self.get_logger().info(
                'Released till the waiting point only) '
                f'robot=[{msg.robot_name}] through_index={msg.release_through_index} '
                f'wait=({wp.x:.2f}, {wp.y:.2f})'
            )
        else:
            self.get_logger().info(
                f'Full horizon released robot=[{msg.robot_name}] '
                f'through_index={msg.release_through_index}'
            )

        self._publish_markers(msg.robot_name, path, msg.release_through_index)

    def _handle_evaluate(
        self,
        request: EvaluatePath.Request,
        response: EvaluatePath.Response,
    ) -> EvaluatePath.Response:
        wps = list(request.waypoints)
        n = len(wps)

        response.success = True
        response.amr_id = request.amr_id
        response.task_id = request.task_id
        response.path_revision = request.path_revision
        response.horizon_m = float(request.query_horizon_m)
        response.dep_from_index = -1
        response.dep_to_index = -1
        response.waiting_for = []

        if n == 0:
            response.status = 'FREE'
            response.message = 'mock ECBS: empty horizon — FREE'
            return response

        indices = [
            _waypoint_path_index(w.index, i) for i, w in enumerate(wps)
        ]
        first_idx = indices[0]

        leg_key = (request.amr_id, request.task_id, request.path_revision)
        hold_idx = self._wait_hold_index.get(leg_key)

        if n < 3:
            response.status = 'FREE'
            response.message = 'mock ECBS: short horizon — FREE'
            self.get_logger().info(
                f'ECBS FREE amr=[{request.amr_id}] task=[{request.task_id}] '
                f'horizon_waypoints={n}'
            )
            return response

        mid = n // 2
        wait_i = max(0, mid - 1)
        mid_idx = indices[mid]
        wait_idx = indices[wait_i]
        mid_wp = wps[mid]
        wait_wp = wps[wait_i]

        # After a WAIT, stay WAIT until the horizon starts past the hold index (re-eval / move).
        if hold_idx is not None and first_idx <= hold_idx:
            pass  # fall through to WAIT below
        elif hold_idx is not None and first_idx > hold_idx:
            self._wait_hold_index.pop(leg_key, None)
            response.status = 'FREE'
            response.message = 'mock ECBS: past wait point — FREE'
            self.get_logger().info(
                f'ECBS FREE amr=[{request.amr_id}] task=[{request.task_id}] '
                f'horizon_waypoints={n} first_idx={first_idx} past_hold={hold_idx}'
            )
            return response
        elif hold_idx is None and first_idx > wait_idx:
            response.status = 'FREE'
            response.message = 'mock ECBS: no conflict — FREE'
            self.get_logger().info(
                f'ECBS FREE amr=[{request.amr_id}] task=[{request.task_id}] '
                f'horizon_waypoints={n}'
            )
            return response

        self._wait_hold_index[leg_key] = wait_idx

        dep = WaitDependency()
        dep.amr_id = self._blocking_amr
        dep.wait_point = WaitPoint()
        dep.wait_point.x = wait_wp.x
        dep.wait_point.y = wait_wp.y
        dep.wait_point.index = wait_idx
        dep.wait_point.wait_sec = 10.0

        for j in range(mid + 1, min(mid + 3, n)):
            pt = ClearPoint()
            pt.x = wps[j].x
            pt.y = wps[j].y
            dep.clear_points.append(pt)

        response.status = 'WAIT'
        response.message = 'WAIT; ZM must decide then register (mock)'
        response.dep_from_index = mid_idx
        response.dep_to_index = mid_idx
        response.waiting_for = [dep]

        self.get_logger().info(
            f'ECBS WAIT amr=[{request.amr_id}] conflict_index={mid_idx} '
            f'wait_index={wait_idx} wait=({wait_wp.x:.2f}, {wait_wp.y:.2f}) '
            f'blocks=[{self._blocking_amr}]'
        )
        return response

    def _publish_markers(
        self,
        robot_name: str,
        path: List,
        release_through: int,
    ) -> None:
        if not path:
            return

        frame = self._marker_frame
        level = (path[0].level_name or '').strip()
        if self._use_level_frame and level:
            frame = level

        arr = MarkerArray()
        delete = Marker()
        delete.header.frame_id = frame
        delete.ns = ''
        delete.id = 0
        delete.action = Marker.DELETEALL
        arr.markers.append(delete)

        released_pts = []
        held_pts = []
        for i, loc in enumerate(path):
            pt = (loc.x, loc.y, 0.05)
            if i <= release_through:
                released_pts.append(pt)
            else:
                held_pts.append(pt)

        marker_id = 1
        if len(released_pts) >= 2:
            arr.markers.append(
                self._line_strip(
                    robot_name, frame, marker_id, released_pts,
                    1.0, 1.0, 0.0, 0.12, 'released',
                )
            )
            marker_id += 1
        elif len(released_pts) == 1:
            arr.markers.append(
                self._sphere(
                    robot_name, frame, marker_id, released_pts[0],
                    1.0, 1.0, 0.0, 0.25, 'released',
                )
            )
            marker_id += 1

        if len(held_pts) >= 2:
            # Connect last released to first held for continuity
            if released_pts:
                held_line = [released_pts[-1]] + held_pts
            else:
                held_line = held_pts
            arr.markers.append(
                self._line_strip(
                    robot_name, frame, marker_id, held_line,
                    0.1, 0.35, 1.0, 0.08, 'held',
                )
            )
            marker_id += 1
        elif held_pts:
            arr.markers.append(
                self._sphere(
                    robot_name, frame, marker_id, held_pts[0],
                    0.1, 0.35, 1.0, 0.2, 'held',
                )
            )

        for i, loc in enumerate(path):
            released = i <= release_through
            m = Marker()
            m.header.frame_id = frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = f'path_wp_{robot_name}'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = loc.x
            m.pose.position.y = loc.y
            m.pose.position.z = 0.08
            m.scale.x = 0.18
            m.scale.y = 0.18
            m.scale.z = 0.18
            if released:
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 0.0, 0.9
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.35, 1.0, 0.9
            arr.markers.append(m)

        self._last_marker_array = arr
        self._marker_pub.publish(arr)
        self.get_logger().debug(
            f'Markers robot=[{robot_name}] frame=[{frame}] waypoints={len(path)} '
            f'released_through={release_through} markers={len(arr.markers)}'
        )

    def _line_strip(
        self,
        robot_name: str,
        frame: str,
        marker_id: int,
        points: List[Tuple[float, float, float]],
        r: float,
        g: float,
        b: float,
        width: float,
        suffix: str,
    ) -> Marker:
        m = Marker()
        m.header.frame_id = frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = f'path_line_{suffix}_{robot_name}'
        m.id = marker_id
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = width
        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = 0.95
        for x, y, z in points:
            p = Point()
            p.x = x
            p.y = y
            p.z = z
            m.points.append(p)
        return m

    def _sphere(
        self,
        robot_name: str,
        frame: str,
        marker_id: int,
        point: Tuple[float, float, float],
        r: float,
        g: float,
        b: float,
        scale: float,
        suffix: str,
    ) -> Marker:
        m = Marker()
        m.header.frame_id = frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = f'path_line_{suffix}_{robot_name}'
        m.id = marker_id
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = point[0]
        m.pose.position.y = point[1]
        m.pose.position.z = point[2]
        m.scale.x = scale
        m.scale.y = scale
        m.scale.z = scale
        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = 0.95
        return m


def main() -> None:
    rclpy.init()
    node = MockEcbsEvalServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
