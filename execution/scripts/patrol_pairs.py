#!/usr/bin/env python3
"""Publish patrol tasks one by one from a list of place pairs.

Each entry in the task list becomes ONE patrol task over its places, e.g.
the pair (y1, P1) becomes a patrol  y1 -> P1. Tasks are published in order,
one after another: the next task is sent only after the previous one was
answered on /task_api_responses (or the timeout passed) plus --delay seconds.

Task file format (one task per line, '#' starts a comment)
----------------------------------------------------------
    y1, P1                  # pair -> patrol y1 -> P1, dispatcher picks robot
    (P5, z3)                # brackets are optional
    amr2: P2, P18           # 'robot:' prefix -> direct task to that robot
    amr4: y2 P1 z4 rounds=2 # more than two places and rounds also work

Place names are checked against the installed nav graph; a task with an
unknown place, or the same place twice in a row, is skipped and reported.

Examples
--------
    source ~/00_FMS_MAIN/install/setup.bash
    SCRIPTS=~/00_FMS_MAIN/src/execution/scripts

    # tasks from a file, dispatcher chooses the robots, 3 s between tasks
    python3 $SCRIPTS/patrol_pairs.py --file $SCRIPTS/patrol_tasks.txt --delay 3

    # pairs on the command line
    python3 $SCRIPTS/patrol_pairs.py --pairs "y1,P1" "P5,z3" "K1,Inbound"

    # give the tasks to amr1, amr2, amr3 in turn (lines with 'amrX:' keep theirs)
    python3 $SCRIPTS/patrol_pairs.py --file $SCRIPTS/patrol_tasks.txt \\
        --robot-names amr1 amr2 amr3

    # check the list without publishing anything
    python3 $SCRIPTS/patrol_pairs.py --file $SCRIPTS/patrol_tasks.txt --dry-run
"""

from __future__ import annotations

import argparse
import json
import re
import sys
import time
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Sequence, Set

DEFAULT_NAV_GRAPH = Path.home() / (
    '00_FMS_MAIN/install/execution/share/execution/config/nav_graphs/0.yaml'
)

ROBOT_RE = re.compile(r'^\s*([A-Za-z][\w-]*)\s*:\s*(.*)$')
ROUNDS_RE = re.compile(r'\brounds\s*=\s*(\d+)\b', re.IGNORECASE)


@dataclass
class PatrolTask:
    places: List[str]
    robot: Optional[str] = None
    rounds: int = 1
    source: str = ''
    errors: List[str] = field(default_factory=list)


# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------
def parse_task_line(line: str, default_rounds: int, source: str) -> Optional[PatrolTask]:
    text = line.split('#', 1)[0].strip()
    if not text:
        return None

    robot = None
    m = ROBOT_RE.match(text)
    if m and ',' not in m.group(1):
        robot, text = m.group(1), m.group(2)

    rounds = default_rounds
    rm = ROUNDS_RE.search(text)
    if rm:
        rounds = int(rm.group(1))
        text = ROUNDS_RE.sub(' ', text)

    text = text.replace('(', ' ').replace(')', ' ').replace('[', ' ').replace(']', ' ')
    places = [p.strip().strip('"\'') for p in re.split(r'[,\s]+', text) if p.strip()]
    return PatrolTask(places=places, robot=robot, rounds=rounds, source=source)


def load_tasks(args: argparse.Namespace) -> List[PatrolTask]:
    tasks: List[PatrolTask] = []
    if args.file:
        for n, line in enumerate(args.file.read_text(encoding='utf-8').splitlines(), 1):
            t = parse_task_line(line, args.rounds, f'{args.file.name}:{n}')
            if t:
                tasks.append(t)
    for i, pair in enumerate(args.pairs or [], 1):
        t = parse_task_line(pair, args.rounds, f'--pairs #{i}')
        if t:
            tasks.append(t)
    return tasks


def load_known_places(nav_graph: Path) -> Optional[Set[str]]:
    """Waypoint names from the RMF nav graph (None if it cannot be read)."""
    if not nav_graph.is_file():
        return None
    try:
        import yaml
        data = yaml.safe_load(nav_graph.read_text(encoding='utf-8'))
        names: Set[str] = set()
        for level in (data.get('levels') or {}).values():
            for v in level.get('vertices') or []:
                if len(v) > 2 and isinstance(v[2], dict) and v[2].get('name'):
                    names.add(str(v[2]['name']))
        return names
    except Exception as exc:  # noqa: BLE001 — a bad graph must not block publishing
        print(f'warning: could not read {nav_graph}: {exc}', file=sys.stderr)
        return None


def validate(task: PatrolTask, known: Optional[Set[str]]) -> None:
    if len(task.places) < 1:
        task.errors.append('no places')
    if task.rounds < 1:
        task.errors.append(f'rounds must be >= 1 (got {task.rounds})')
    for a, b in zip(task.places, task.places[1:]):
        if a == b:
            task.errors.append(f'same place twice in a row ({a})')
    if task.rounds > 1 and len(task.places) > 1 and task.places[0] == task.places[-1]:
        task.errors.append('first and last place equal with rounds > 1')
    if known is not None:
        unknown = [p for p in task.places if p not in known]
        if unknown:
            task.errors.append(f'unknown place(s): {", ".join(unknown)}')


# ---------------------------------------------------------------------------
# Publishing
# ---------------------------------------------------------------------------
def run_publisher(args: argparse.Namespace, tasks: List[PatrolTask]) -> int:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile
    from rclpy.qos import QoSReliabilityPolicy
    from rmf_task_msgs.msg import ApiRequest, ApiResponse

    class PatrolPublisher(Node):
        def __init__(self) -> None:
            super().__init__('patrol_pairs')
            qos = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.pub = self.create_publisher(ApiRequest, 'task_api_requests', qos)
            self.create_subscription(
                ApiResponse, 'task_api_responses', self._on_response, qos)
            self._pending_id: Optional[str] = None
            self._result: Optional[dict] = None

        def _on_response(self, msg: ApiResponse) -> None:
            if self._pending_id and msg.request_id == self._pending_id:
                try:
                    self._result = json.loads(msg.json_msg)
                except json.JSONDecodeError:
                    self._result = {'raw': msg.json_msg}

        def wait_for_listeners(self, timeout: float) -> bool:
            # The RMF dispatcher / fleet adapter only receive requests sent
            # AFTER they matched this publisher — wait for them first.
            deadline = time.time() + timeout
            while rclpy.ok() and time.time() < deadline:
                if self.pub.get_subscription_count() > 0:
                    rclpy.spin_once(self, timeout_sec=0.5)  # settle discovery
                    return True
                rclpy.spin_once(self, timeout_sec=0.2)
            return False

        def now_ms(self) -> int:
            return int(time.time() * 1000)

        def submit(self, task: PatrolTask, robot: Optional[str]) -> Optional[dict]:
            request = {
                'category': 'patrol',
                'unix_millis_earliest_start_time': 0,
                'unix_millis_request_time': self.now_ms(),
                'requester': 'patrol_pairs',
                'description': {'places': task.places, 'rounds': task.rounds},
            }
            if robot:
                payload = {'type': 'robot_task_request', 'robot': robot,
                           'fleet': args.fleet, 'request': request}
            else:
                payload = {'type': 'dispatch_task_request', 'request': request}

            req_id = f'patrol_pairs_{uuid.uuid4().hex[:12]}'
            msg = ApiRequest()
            msg.request_id = req_id
            msg.json_msg = json.dumps(payload)
            if args.verbose:
                print(json.dumps(payload, indent=2))

            self._pending_id, self._result = req_id, None
            self.pub.publish(msg)
            deadline = time.time() + args.timeout
            while rclpy.ok() and self._result is None and time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
            self._pending_id = None
            return self._result

    rclpy.init()
    node = PatrolPublisher()
    try:
        if not node.wait_for_listeners(args.wait_listeners):
            print('error: nobody is subscribed to /task_api_requests — is the RMF '
                  'dispatcher / fleet adapter running?', file=sys.stderr)
            return 1

        robots = list(args.robot_names or [])
        sent = accepted = 0
        for i, task in enumerate(tasks):
            robot = task.robot or (robots[i % len(robots)] if robots else None)
            label = ' -> '.join(task.places)
            who = robot or 'dispatcher'
            print(f'[{i + 1}/{len(tasks)}] patrol {label} (rounds={task.rounds}) -> {who}')
            result = node.submit(task, robot)
            sent += 1
            if result is None:
                print(f'    no response within {args.timeout:.0f}s')
            elif result.get('success'):
                accepted += 1
                booking = ((result.get('state') or {}).get('booking') or {}).get('id', '?')
                print(f'    accepted: task {booking}')
            else:
                print(f'    REJECTED: {json.dumps(result.get("errors", result))}')
            if i + 1 < len(tasks) and args.delay > 0:
                time.sleep(args.delay)
        print(f'Done: {sent} sent, {accepted} accepted.')
        return 0 if accepted == sent else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ---------------------------------------------------------------------------
def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description='Publish patrol tasks one by one from a list of place pairs.')
    p.add_argument('--file', '-f', type=Path, help='task list file (one task per line)')
    p.add_argument('--pairs', nargs='+', metavar='"A,B"',
                   help='tasks on the command line, e.g. "y1,P1" "P5,z3"')
    p.add_argument('--robot-names', nargs='+', metavar='AMR',
                   help='assign tasks to these robots in turn (default: dispatcher picks)')
    p.add_argument('--fleet', default='amr_fleet')
    p.add_argument('--rounds', type=int, default=1, help='default rounds per task')
    p.add_argument('--delay', type=float, default=2.0,
                   help='seconds between tasks after a response (default 2)')
    p.add_argument('--timeout', type=float, default=10.0,
                   help='seconds to wait for each response (default 10)')
    p.add_argument('--wait-listeners', type=float, default=15.0,
                   help='seconds to wait for the RMF subscribers (default 15)')
    p.add_argument('--nav-graph', type=Path, default=DEFAULT_NAV_GRAPH,
                   help='nav graph used to validate place names')
    p.add_argument('--no-validate', action='store_true', help='skip place-name check')
    p.add_argument('--dry-run', action='store_true', help='parse and validate only')
    p.add_argument('-v', '--verbose', action='store_true', help='print each payload')
    args = p.parse_args(argv)
    if not args.file and not args.pairs:
        p.error('give --file and/or --pairs')
    if args.file and not args.file.is_file():
        p.error(f'task file not found: {args.file}')
    return args


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(list(sys.argv[1:] if argv is None else argv))
    tasks = load_tasks(args)
    if not tasks:
        print('No tasks found.', file=sys.stderr)
        return 1

    known = None if args.no_validate else load_known_places(args.nav_graph)
    if known is None and not args.no_validate:
        print(f'warning: place names not validated ({args.nav_graph} unreadable)',
              file=sys.stderr)

    valid: List[PatrolTask] = []
    for t in tasks:
        validate(t, known)
        if t.errors:
            print(f'SKIP {t.source}: {" -> ".join(t.places)}  ({"; ".join(t.errors)})')
        else:
            valid.append(t)
    print(f'{len(valid)} of {len(tasks)} task(s) valid.')
    for i, t in enumerate(valid, 1):
        who = t.robot or (f'(round-robin {args.robot_names})' if args.robot_names
                          else 'dispatcher')
        print(f'  {i:2d}. {" -> ".join(t.places)}  rounds={t.rounds}  robot={who}')

    if args.dry_run or not valid:
        return 0 if valid else 1
    return run_publisher(args, valid)


if __name__ == '__main__':
    sys.exit(main())
