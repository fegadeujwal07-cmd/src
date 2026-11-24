# `mutex_supervisor/main.py` — Detailed walkthrough (with line ranges)

This document explains how the mutex supervisor works by walking through
`mutex_supervisor/main.py` in order, referencing **line ranges** from that file.

File: `src/rmf/rmf_ros2/mutex_supervisor/mutex_supervisor/main.py`

> Notes
>
> - Line numbers refer to the **current version** of the file in this workspace.
> - The supervisor uses **mutex group names** (e.g. `M1`) as the lock key. All robots
>   that traverse the same mutex name share the same lock, even if their exact
>   entry/exit coordinates differ.

---

## 1) Imports and node class

**L1–L20**: Imports.

- `rclpy` + `Node`: ROS 2 node base class.
- `QoSProfile` + policies: used to match RMF topic QoS (reliable).
- `deque`: used for FIFO queue per mutex.
- Messages:
  - `rmf_traffic_msgs/msg/ItinerarySet`: planned trajectories per RMF participant.
  - `rmf_fleet_msgs/msg/FleetState`: live robot positions.
  - `rmf_fleet_msgs/msg/PauseRequest`: pause/resume command to the fleet adapter/driver.
  - `rmf_fleet_msgs/msg/MutexGroupRequestStatus`: custom status stream published by this node.

**L22–L23**: `class MutexSupervisor(Node)` defines the ROS node.

---

## 2) `__init__`: parameters, parsing, state, subscriptions, publishers

### 2.1 Parameters and config

**L25–L48**:

- Declares and reads parameters:
  - `nav_graph_path`: **required** path to `0.yaml`.
  - `match_tolerance`: distance threshold (meters) for matching itinerary waypoints to mutex vertices from `0.yaml`.
  - `claim_distance`: distance (meters) to begin mutex claim logic.
  - `exit_reached_distance`: distance (meters) to decide the robot has reached mutex exit.
  - `clearance_distance`: distance (meters) to decide the robot has cleared the mutex exit (and can unlock).
- `command_cooldown` is an internal (not parameterized) rate limit for pause/resume publishing.

If `nav_graph_path` is empty, the node throws `ValueError` (startup fails).

### 2.2 Parse mutex groups from `0.yaml`

**L49–L71**:

- Imports `MutexZoneParser` from `mutex_parser.py`.
- `parser.parse_mutex_zones()` returns a dict like:

  - `mutex_zones['M1']['vertices'] = [(x1,y1), (x2,y2), ...]`

This parser only extracts mutex-tagged vertices; it does **not** decide entry/exit.
Entry/exit are decided later from the robot’s itinerary (path order).

### 2.3 In-memory state used by the supervisor

**L73–L109**:

- `self.robot_states`: main per-participant state store. Each participant’s dict contains:
  - `itinerary_points`: the extracted (x,y) list from `ItinerarySet`.
  - `mutex_zones`: intersection dict for mutexes on this plan (entry/exit/index per mutex).
  - `mutex_status`: per-mutex runtime status (`NORMAL`, `APPROACHING`, `PASSING_MUTEX`, `QUEUED_STOPPING`, `STOPPED_WAITING`) plus flags.
  - `active_mutex`: mutex currently being processed (closest, not-yet-cleared).

- Mapping:
  - `robot_name_to_participant` and `participant_to_robot_name` are maintained **from `ItinerarySet.robot_name`**.

- Live info from `/fleet_states`:
  - `robot_name_to_fleet`: used to fill `PauseRequest.fleet_name`.
  - `robot_positions`: latest `(x,y)` per robot.

- Status publish rate limiting:
  - `last_publish_time[(robot_name, mutex_name)]`
  - `publish_interval`

- Per-robot command tracking:
  - `robot_pause_tracking[robot_name]` stores `mode_request_id`, flags, timestamps.

- Per-mutex lock state:
  - `mutex_states[mutex_name] = {'locked': bool, 'owner': robot_name|None, 'queue': deque([robot_name,...])}`

### 2.4 QoS + subscriptions + publishers

**L110–L157**:

- Builds a reliable QoS profile (depth 10).
- Subscribes:
  - `/rmf_traffic/itinerary_set` → `_itinerary_set_callback` (plan-level info)
  - `/fleet_states` → `_fleet_states_callback` (live positions)
- Publishes:
  - `/robot_pause_requests` (`PauseRequest`) for pause/resume control
  - `/mutex_group_request` (`MutexGroupRequestStatus`) for visualization/monitoring

---

## 3) Geometry helpers + itinerary parsing

### 3.1 Euclidean distance

**L159–L161**: `_euclidean_distance(p1, p2)` returns \(\sqrt{dx^2 + dy^2}\).

### 3.2 Find closest waypoint index (used for matching)

**L163–L195**: `_find_closest_waypoint_index(target, waypoints, tolerance)`

- Scans all waypoint coords, picks the closest, returns its index if within `tolerance`.

### 3.3 Extract itinerary points (x,y) from `ItinerarySet`

**L197–L229**: `_extract_itinerary_points(msg)`

- Iterates all routes in `msg.itinerary`.
- Iterates each trajectory waypoint.
- Extracts `(x,y)` from waypoint `position` and appends to `itinerary_points`.

This is the “flattened” list of (x,y) used for mutex intersection detection.

---

## 4) Detect mutex intersections in the itinerary (entry/exit per robot plan)

**L231–L319**: `_detect_mutex_intersections(itinerary_points)`

For each mutex group (e.g. `M1`):

- Loops through each itinerary point and checks if it is within `match_tolerance`
  of **any** mutex vertex from `0.yaml`.
- Records all matching itinerary indices and their coordinates.

Then it chooses:

- `entry_index`: the first matching index in the robot’s path order.
- `exit_index`: the last matching index in the robot’s path order.
- `entry_coord` / `exit_coord`: taken from the itinerary coordinates (not the graph vertex list).

Returns:

```python
{
  "M1": {
    "entry_index": ...,
    "exit_index": ...,
    "entry_coord": (x, y),
    "exit_coord": (x, y),
  },
  ...
}
```

---

## 5) `/rmf_traffic/itinerary_set` callback: store plan + map participant↔robot

**L321–L515**: `_itinerary_set_callback(msg)`

### 5.1 Basic extraction and mutex intersection detection

**L328–L350**:

- Reads `participant_id` and `plan_id`.
- Extracts `itinerary_points`.
- Computes `mutex_intersections`.

### 5.2 Explicit mapping via `ItinerarySet.robot_name`

**L351–L376**:

- Reads `msg.robot_name`.
- If non-empty:
  - `participant_to_robot_name[participant_id] = robot_name`
  - `robot_name_to_participant[robot_name] = participant_id`
  - Logs mapping (and warns if overwritten).
- If empty:
  - Logs a warning and leaves the participant unmapped.

### 5.3 Initialize participant state or reset on new plan

**L378–L486**:

- If first time seeing this participant: creates the `robot_states[participant_id]` dict.
- If participant already exists: logs “new plan”, resets:
  - exit flags (`exit_reached`, `exit_cleared`)
  - status to `NORMAL`
  - pause/resume tracking flags
  - special safety: if the participant was owner of a mutex that disappeared from the new plan, it releases the mutex to prevent deadlock (and grants next queued robot if any).

### 5.4 Store latest itinerary + intersection results

**L482–L509**:

- Always updates:
  - `itinerary_points`
  - `mutex_zones` (the intersection dict)
  - `active_mutex` reset to None
- Logs whether this plan passes through mutex groups and the entry→exit indices.

---

## 6) `/fleet_states` callback: live position, update mutex state machine, publish commands

**L516–L595**: `_fleet_states_callback(msg)`

For each robot in the fleet:

**L532–L557**:

- Tracks `robot_name_to_fleet[robot_name] = fleet_name`
- Stores `robot_positions[robot_name] = (x,y)`
- Ensures `robot_pause_tracking[robot_name]` exists.

**L559–L570**:

- Looks up `participant_id = robot_name_to_participant.get(robot_name)`.
- If there is no mapping yet:
  - Runs `_process_robot_mutex_status_by_name(...)` (best-effort), then continues.

**L572–L585**:

- If mapped and `participant_id` exists in `robot_states`:
  - Updates live pose/level/task_id.
  - Calls `_process_robot_mutex_status(...)` (core state machine).
  - Calls `_handle_pause_resume_commands(...)` (periodic pause commands for queued robots).

---

## 7) Selecting the active mutex for a robot

**L596–L633**: `_select_active_mutex(mutex_zones, mutex_status, robot_pose)`

- Chooses the not-yet-cleared mutex with smallest distance to its `entry_coord`.
- Returns mutex name or `None`.

---

## 8) Core mutex state machine (per robot, per fleet_states tick)

**L635–L858** and **L680–L835**: `_process_robot_mutex_status(participant_id, robot_name, robot_pose)`

Key steps:

### 8.1 Initialize per-mutex status for this robot

**L683–L746**:

- If this robot has a mutex in `mutex_zones` and no `mutex_status[mutex_name]` entry yet:
  - Computes `pre_mutex_index = entry_index - 1` (or 0).
  - Gets `pre_mutex_coord` from `itinerary_points[pre_mutex_index]`.
  - Initializes status dict:
    - `status` initially `NORMAL` or `APPROACHING` if already within `claim_distance`.
    - stores `entry_index`, `exit_index`, `pre_mutex_index`, `pre_mutex_coord`.
    - exit flags false.

### 8.2 Transition NORMAL → APPROACHING and attempt claim/queue

**L768–L797** (and earlier portions not shown in snippet):

- If `status == NORMAL` and `dist_to_entry <= claim_distance`:
  - Transitions to `APPROACHING`.
  - Calls `_handle_mutex_claim(robot_name, mutex_name)` which:
    - claims if free → owner set
    - else queues robot
  - Publishes `/mutex_group_request` status.

### 8.3 Ensure claim logic runs if APPROACHING but unmapped in mutex_state

**L788–L797**:

- If robot is APPROACHING but neither owner nor queued, calls `_handle_mutex_claim` again.

### 8.4 Transition to PASSING_MUTEX and resume if it becomes owner

**L798–L811**:

- If robot was approaching/queued and now mutex owner matches robot:
  - Transition to `PASSING_MUTEX`.
  - If it was queued, sends resume: `_send_resume_request(robot_name, mutex_name)`.

### 8.5 Exit reached / exit cleared for owner only

**L812–L835**:

- Only when `mutex_state.owner == robot_name`:
  - Exit reached when `dist_to_exit <= exit_reached_distance`
  - Exit cleared when `dist_to_exit >= clearance_distance`
  - On cleared: `_handle_mutex_unlock(...)` releases and grants next queued robot.

### 8.6 Publish unified status

**L845–L857**:

- Publishes `/mutex_group_request` if within claim distance and robot is owner or queued.

---

## 9) Claim/queue logic for a mutex name

**L859–L963**: `_handle_mutex_claim(robot_name, mutex_name)`

- If mutex is not locked:
  - lock it, set owner, update status `PASSING_MUTEX`, publish owner status.
- Else:
  - if already owner: do nothing
  - else:
    - append to `mutex_state['queue']` if not present
    - set status `QUEUED_STOPPING` then `STOPPED_WAITING`
    - send pause request:
      - `TYPE_PAUSE_AT_CHECKPOINT` (1) toward `pre_mutex_index`

---

## 10) Unlock and grant next queued robot

**L964–L1012**: `_handle_mutex_unlock(robot_name, mutex_name)`

- Verifies the caller is the owner.
- Sets `locked=False`, `owner=None`.
- If queue not empty:
  - pops next robot
  - sets it as owner and locks mutex again
  - updates its status to `PASSING_MUTEX`
  - sends resume request (type 2)
  - publishes updated status for the new owner.

---

## 11) Pause requests (type 1 and type 0)

**L1042–L1129**: `_send_pause_request(robot_name, mutex_name, immediate=False)`

- Rate-limited by `command_cooldown`.
- Builds `PauseRequest`:
  - If `immediate=False`: `TYPE_PAUSE_AT_CHECKPOINT` (1) + `at_checkpoint = pre_mutex_index`
  - If `immediate=True`: `TYPE_PAUSE_IMMEDIATELY` (0) + `at_checkpoint = 0`

This is used while queued:

- type 1 continuously while approaching the pre-mutex waypoint
- type 0 continuously when within 0.5m of the pre-mutex waypoint, still queued

---

## 12) Resume requests (type 2)

**L1131–L1191**: `_send_resume_request(robot_name, mutex_name)`

- Rate-limited + debounce (won’t publish duplicates).
- Publishes `PauseRequest.TYPE_RESUME` (2) once when granted.

---

## 13) Periodic pause publishing while queued

**L1193–L1231**: `_handle_pause_resume_commands(robot_name, state)`

- Runs every `/fleet_states` tick for mapped robots.
- Guard: returns immediately if the robot is the mutex **owner** (so owners don’t get pause commands).
- If queued (`QUEUED_STOPPING` or `STOPPED_WAITING`):
  - Measures distance to `pre_mutex_coord`.
  - If <= 0.5m: sends type 0 (immediate stop), else type 1 (checkpoint decel).

---

## 14) Unified status publishing `/mutex_group_request`

**L1263–L1379**: `_publish_mutex_group_request_status(...)`

- Converts internal status into one of:
  - `REQUESTING`, `QUEUED`, `OWNER`, `NORMAL`
- Computes queue position if queued.
- Rate-limits to once per second per (robot, mutex).
- Publishes a structured `MutexGroupRequestStatus` message.

---

## 15) Helper for “distance to entry” used by claim/unlock publishing

**L1381–L1404**: `_get_robot_dist_to_entry(robot_name, mutex_name)`

- Finds the robot’s `current_pose` and `entry_coord` for the mutex from `robot_states`.
- Returns Euclidean distance or `None`.

---

## 16) Node entry point

**L1407–L1422**: `main()`:

- `rclpy.init`, instantiate node, `rclpy.spin`, cleanup on shutdown.

---

## 17) `PauseRequest.type` semantics on `/robot_pause_requests`

The supervisor publishes `rmf_fleet_msgs/msg/PauseRequest` on `/robot_pause_requests`.
The **`type` field** is interpreted as:

- **`0` – `TYPE_PAUSE_IMMEDIATELY`**
  - Used only when a robot is **queued** for a mutex and is within **0.5 m** of its pre-mutex waypoint.
  - Indicates a hard/immediate stop at the robot’s current pose.
  - Set in `_send_pause_request(..., immediate=True)`, which is called from `_handle_pause_resume_commands(...)` when `dist_to_pre_mutex <= 0.5` and the robot is still in a queued state (`QUEUED_STOPPING` / `STOPPED_WAITING`).

- **`1` – `TYPE_PAUSE_AT_CHECKPOINT`**
  - Used while the robot is **queued** for a mutex but still farther than 0.5 m from the pre-mutex waypoint.
  - `at_checkpoint` is set to the **pre-mutex waypoint index** (`pre_mutex_index`), so the controller can decelerate toward that point.
  - Published:
    - Immediately when the robot is first queued in `_handle_mutex_claim(...)`.
    - Periodically (respecting `command_cooldown`) while the robot remains queued and is not the mutex owner, via `_handle_pause_resume_commands(...)`.

- **`2` – `TYPE_RESUME`**
  - Used when the robot is **granted** the mutex (transitions from queued state to `PASSING_MUTEX`).
  - Published once in `_send_resume_request(...)`, which is called:
    - When a queued robot becomes owner in `_process_robot_mutex_status(...)`.
    - When a queued robot is granted the mutex after a different owner unlocks in `_handle_mutex_unlock(...)`.

Owners of a mutex **never receive new pause commands** from the supervisor:

- `_handle_pause_resume_commands(...)` explicitly returns early if `mutex_states[active_mutex]['owner'] == robot_name`.


