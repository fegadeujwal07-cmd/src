# Mutex Supervisor

External mutex supervisor for Nav2-based fleet management with RMF. This node monitors robot plans and positions to enforce mutex zone access control, ensuring only one robot traverses a critical segment at a time.

## Overview

The mutex supervisor provides external mutex zone management for RMF-based fleets. It:

- **Detects mutex zones** from navigation graph YAML files (waypoint-based entry/exit coordinates)
- **Monitors robot plans** via `/rmf_traffic/itinerary_set` to detect mutex intersections
- **Tracks robot positions** via `/fleet_states` for live execution monitoring
- **Enforces FIFO queue** for mutex access when multiple robots approach
- **Publishes pause/resume commands** via `/robot_pause_requests` to control robot motion

## Required Topics

### Subscriptions

- `/rmf_traffic/itinerary_set` (rmf_traffic_msgs/ItinerarySet)
  - RMF-planned trajectories for robots
  - Used to detect which robots will pass through mutex zones

- `/fleet_states` (rmf_fleet_msgs/FleetState)
  - Live robot positions and states
  - Used for proximity-based claim/unlock logic and pause/resume timing

### Publications

- `/robot_pause_requests` (rmf_fleet_msgs/PauseRequest)
  - Commands to pause/resume robots
  - Published when robots are queued (pause) or granted mutex access (resume)

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `nav_graph_path` | string | **required** | Path to navigation graph YAML file (e.g., `0.yaml`) |
| `match_tolerance` | double | 0.5 | Tolerance (meters) for matching mutex entry/exit coordinates to itinerary waypoints |
| `claim_distance` | double | 4.0 | Distance (meters) from mutex entry to trigger claim logic |
| `exit_reached_distance` | double | 0.7 | Distance (meters) to mutex exit to detect exit reached |
| `clearance_distance` | double | 0.9 | Distance (meters) beyond mutex exit to ensure full clearance before releasing |

## Usage

### Launch File

```bash
ros2 launch mutex_supervisor mutex_supervisor.launch.xml \
    nav_graph_path:=/path/to/0.yaml \
    match_tolerance:=0.5 \
    claim_distance:=4.0 \
    exit_reached_distance:=0.7 \
    clearance_distance:=0.9
```

### Using Parameters File

```bash
ros2 launch mutex_supervisor mutex_supervisor.launch.xml \
    nav_graph_path:=/path/to/0.yaml \
    params_file:=/path/to/mutex_supervisor_params.yaml
```

## Mutex Zone Definition

Mutex zones are defined in the navigation graph YAML file by tagging vertices with `mutex: <name>`. The supervisor:

1. Discovers all vertices with the same mutex name
2. Orders them along the corridor (by axis with largest spread)
3. Uses first vertex as **entry** and last vertex as **exit**

Example from `0.yaml`:
```yaml
vertices:
  - [9.3, -0.9, {mutex: M1, name: ''}]  # Entry
  - [8.8, -0.9, {mutex: M1, name: ''}]
  - [8.3, -0.9, {mutex: M1, name: ''}]
  - [7.8, -0.9, {mutex: M1, name: ''}]
  - [7.2, -0.9, {mutex: M1, name: ''}]
  - [6.6, -0.9, {mutex: M1, name: ''}]  # Exit
```

## How It Works

1. **Plan Detection (Step 3)**: When an itinerary is published, the supervisor extracts waypoints and matches them against mutex entry/exit coordinates (within `match_tolerance`).

2. **Approach Detection (Step 4)**: As robots move, the supervisor computes distance to mutex entry. When within `claim_distance`, the robot attempts to claim the mutex.

3. **Claim Logic (Step 5)**:
   - If mutex is **free**: Robot claims it, status → `PASSING_MUTEX`
   - If mutex is **locked**: Robot is queued (FIFO), status → `QUEUED_STOPPING` → `STOPPED_WAITING`, pause command sent

4. **Unlock Logic (Step 5)**: When owner robot clears exit (moves `clearance_distance` beyond exit), mutex is released and next robot in queue is granted.

5. **Command Publishing (Step 6)**: Pause commands are sent when robots are queued; resume commands are sent when mutex is granted.

## Status Transitions

- `NORMAL` → `APPROACHING` (when within `claim_distance` of entry)
- `APPROACHING` → `PASSING_MUTEX` (when mutex claimed)
- `APPROACHING` → `QUEUED_STOPPING` (when mutex locked by another)
- `QUEUED_STOPPING` → `STOPPED_WAITING` (after pause sent)
- `STOPPED_WAITING` → `PASSING_MUTEX` (when mutex granted, resume sent)

## Limitations

1. **Mutex Zone Assumptions**:
   - Assumes mutex vertices form a linear corridor (ordered along one axis)
   - Does not handle branched or complex mutex geometries
   - Entry/exit are determined by first/last vertex in sorted order

2. **Distance-Based Approach**:
   - Uses Euclidean distance for all proximity checks
   - Does not account for path geometry or obstacles
   - Assumes robots follow planned waypoints closely

3. **Robot Name Mapping**:
   - Relies on `participant_id` ↔ `robot_name` mapping from `/fleet_states`
   - Uses simple inference when mapping is uncertain (warns on first occurrence)
   - May need refinement based on your fleet adapter implementation

4. **No Re-routing**:
   - Does not modify RMF plans (itinerary_set is read-only)
   - Only pauses/resumes robots, does not suggest alternative routes

5. **Single Active Mutex**:
   - For robots with multiple mutex zones, only processes the closest (not-yet-cleared) mutex
   - Other mutexes are queued until the active one is cleared

## Safety Features

- **Mapping Stability**: Warns when `participant_id` ↔ `robot_name` mapping is uncertain, prefers `robot_name` as primary key
- **State Reset**: Resets mutex status flags when new `itinerary_set` arrives (new plan)
- **Command Cooldown**: 0.5s cooldown between pause/resume commands per robot to prevent spam
- **Debouncing**: Tracks `pause_sent`/`resume_sent` flags to prevent duplicate commands

## Testing

See the test procedure in the launch file comments or run:

```bash
# Terminal 1: Start RMF + fleet adapter
# (your usual RMF startup commands)

# Terminal 2: Start mutex supervisor
ros2 launch mutex_supervisor mutex_supervisor.launch.xml \
    nav_graph_path:=/path/to/0.yaml

# Terminal 3: Monitor topics
ros2 topic echo /rmf_traffic/itinerary_set -n 1
ros2 topic echo /fleet_states -n 1
ros2 topic echo /robot_pause_requests

# Send delivery tasks that cause both robots to use mutex M1
# Expected sequence:
# - robot1 claims mutex
# - robot2 queued + pause published
# - robot1 clears exit
# - robot2 resumed + granted mutex
```

## License

Apache License 2.0

