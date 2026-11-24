# `ecbs_planner` — Package README (code detail)

Wait-only ECBS library and msgs used by the fleet adapter’s `EcbsGate`.  
Fleet-adapter wiring is documented in [`../../CHANGES_DETAIL_README.md`](../../CHANGES_DETAIL_README.md).

**Production:** `EcbsGate` links `ecbs_planner_core` **in-process**.  
**Optional:** standalone `ecbs_planner_node` (topic pipeline) — not required when the gate is on.

---

## 1. Package layout

```
src/ecbs_planner/
├── package.xml
├── CMakeLists.txt
├── README.md                 ← this file
├── CHANGES_README.md         ← short pointer
├── ../ECBS_PLANNER.md        ← task list F1–F8 (src/ root)
├── docs/ACCEPTANCE.md        ← live tests still required
├── config/ecbs_params.yaml   ← solver params (SolverConfig defaults match)
├── msg/   (15)  ClearPoint WaitPoint WaitDependency PathPoint ToZoneManager
│                EcbsReplan EvaluatePathWaypoint FromZoneManager RegisterUpdate
│                RouteOccurrence ConflictRecord MovementProposal GrantEvent
│                ClearanceEvent AuthorityConflict
├── srv/   (3)   EvaluatePath BaseExtensionQuery CommitProposal
├── include/ecbs_planner/     ← one header per source below + types / path_* helpers
├── src/   (16 .cpp; 12 in ecbs_planner_core)
│   core lib:  schedule_cache  ecbs_solver  evaluate_path  nav_graph_junctions
│              conflict_store  passage_arbitration  proposal_engine  passage_action
│              action_compatibility  dependency_graph  conflict_group_builder
│              group_passage_scheduler
│   executables: main + ecbs_node (ecbs_planner_node), to_zone_viz_node,
│                evaluate_path_server_main (evaluate_path_server)
├── launch/ scripts/
└── test/  test_ecbs_solver  test_conflict_store  test_passage_arbitration
           test_proposal_engine  test_group_passage_scheduler
           test_solver_config_defaults
```

**Build products**

| Target | Role |
|--------|------|
| `ecbs_planner` (msgs) | ROS 2 interfaces |
| `ecbs_planner_core` | Shared lib: cache, solver, conflict store, proposal engine, group scheduler (linked by `rmf_fleet_adapter`) |
| `ecbs_planner_node` | Optional executable |
| `evaluate_path_server` | EvaluatePath / BaseExtensionQuery / CommitProposal service node |
| `to_zone_manager_viz` | RViz markers for `/to_zone_manager` |
| `test_*` (6 targets) | GTest, one per test file in `test/` |

---

## 2. `package.xml`

| Item | Detail |
|------|--------|
| Name / version | `ecbs_planner` / `0.1.0` |
| Build | `ament_cmake`, `rosidl_default_generators` |
| Depends | `rclcpp`, `builtin_interfaces`, `geometry_msgs`, `visualization_msgs`, `rmf_fleet_msgs`, `rmf_traffic_msgs`, `rmf_building_map_msgs`, `ament_index_cpp`, `yaml-cpp`, `monitor` |
| Export | `rosidl_interface_packages` member; msgs + runtime |
| Note | The solver is **not FIFO** (`ecbs_solver.hpp`: waiting_for on both AMRs, no FIFO preference); the `package.xml` description is correct on that point. |

---

## 3. `CMakeLists.txt` (what each part builds)

1. **`rosidl_generate_interfaces`** — generates C++/Python types for the 15 `.msg` and 3 `.srv` files.  
2. **`add_library(ecbs_planner_core SHARED …)`** — the 12 core `.cpp` files listed above, C++17, public include `include/`.  
3. **`add_executable(ecbs_planner_node …)`** — `main.cpp` + `ecbs_node.cpp`; links core + msg typesupport.  
4. **Install / export** — headers, `ecbs_planner_core`, alias `ecbs_planner::ecbs_planner_core`, launch, node binary.  
5. **Tests** — one `ament_add_gtest` per file in `test/`, each linked to core.

Dependents (`rmf_fleet_adapter`) should:

```cmake
find_package(ecbs_planner REQUIRED)
target_link_libraries(... ecbs_planner::ecbs_planner_core)
```

---

## 4. ROS messages (`msg/`)

Lean payload for `/to_zone_manager` and replan.

### `ClearPoint.msg`
```
float64 x
float64 y
```
A map point the blocker must clear before the waiter proceeds.

### `WaitDependency.msg`
```
string amr_id
ClearPoint[] clear_points
```
“I am waiting for this AMR until these points are clear.”

### `PathPoint.msg`
```
builtin_interfaces/Time t
float64 x
float64 y
float64 wait_sec
WaitDependency[] waiting_for
```
One timed waypoint. `wait_sec` is dwell/hold; `waiting_for` explains **who** and **where**.

### `ToZoneManager.msg`
```
string fleet_name
string amr_id
PathPoint[] path
```
Published on `/to_zone_manager` after a successful solve (one full path per accept policy in the gate).

### `EcbsReplan.msg`
```
builtin_interfaces/Time stamp
string amr_id                # candidate (replans)
string task_id
string reason
string[] conflicting_amrs
string decision              # REPLAN | NO_ALTERNATIVE | CLEAR ("" = resolver report)
string keep_amr              # keeps its path
int64[] blocked_waypoints    # nav-graph indices closed for amr_id only
float64[] blocked_until_sec
uint32 revision
```
Published on `/ecbs_replan_candidate` (`replan_topic`) by two sources that
share one publisher:

- `ReplanCoordinator` (`replan_candidate.cpp`): head-on / crossing mutual
  holds, a parked AMR on a path, or an earlier AMR's goal on a later path —
  REPLAN only if the detour beats waiting (`decision` set).
- `GlobalResolver`: a deadlock cycle no wait order can break (`decision`
  empty, informational; `reason` carries the conflict id).

---

## 5. `include/ecbs_planner/types.hpp` — internal types

C++ mirrors used by cache + solver (not ROS msgs).

| Type | Fields / meaning |
|------|------------------|
| `ClearPoint` | `x`, `y` |
| `WaitDependency` | `amr_id`, `clear_points` |
| `TimedPoint` | `t_sec`, `x`, `y`, `yaw`, `level_name`, optional graph `index`, `wait_duration_sec`, `waiting_for` |
| `TimedPath` | `amr_id`, `fleet_name`, `task_id`, `dispatch_task_id`, `phase_index`, `path_revision`, `waypoints` |
| `ConflictHit` | `other_amr`, `self_wp`, `other_wp`, `overlap_start_sec`, `other_clear_sec`, `self_clear_sec` |
| `SolveStatus` | `Ok`, `UpdatedOther`, `FailReplan` |
| `SolveResult` | `status`, `path`, `updated_others`, `reason`, `conflicting_amrs` |

`UpdatedOther` means the candidate stayed conflict-free only after delaying an **already-cached** AMR (FIFO fallback when the candidate cannot wait).

---

## 6. Schedule cache

### Header — `schedule_cache.hpp`

Thread-safe map `amr_id → TimedPath`.

| Method | Behaviour |
|--------|-----------|
| `upsert(path)` | Insert/replace by `amr_id` (no-op if `amr_id` empty) |
| `erase(amr_id)` | Remove one AMR |
| `clear()` | Empty table |
| `get(amr_id)` | Optional copy of one path |
| `all_except(exclude)` | All paths, optionally skipping one AMR |
| `size()` | Count of AMRs |

### Implementation — `schedule_cache.cpp`

Every public method takes `std::lock_guard` on `mutex_`.  
In production, `EcbsGate` commits paths here after a successful `solve` (and when applying waits to others).

---

## 7. Wait-only solver

### Header — `ecbs_solver.hpp`

**`SolverConfig`**

| Field | Default | Role |
|-------|---------|------|
| `safety_margin_sec` | `2.0` | Extra clearance after other clears |
| `max_wait_sec` | `30.0` | Cap on cumulative wait |
| `max_iters` | `20` | Loop limit |
| `position_epsilon_m` | `0.35` | XY match if no graph index |

**Policy (current)**

1. **FIFO** when both can wait → **candidate** (later / not yet committed) waits.  
2. If only the cached AMR can wait within `max_wait` → that AMR waits (`UpdatedOther`).  
3. If neither can → `FailReplan`.

### Implementation — `ecbs_solver.cpp`

#### Anonymous helpers

| Function | Role |
|----------|------|
| `intervals_overlap(a0,a1,b0,b1)` | Time-interval clash |
| `clear_points_from(blocker, wp)` | Conflict cell + next cell as clear points |
| `add_wait_dependency(wait_wp, blocker, pts)` | Merge or append `WaitDependency` |
| `delay_arrival_at(...)` | Add wait at wp before conflict (or wp0); bump later `t_sec`; attach deps |

#### `same_cell`

- If both have `index` → indices equal.  
- Else if levels differ → false.  
- Else XY distance ≤ `eps_m`.

#### `depart_sec(path, i)`

Occupancy end at waypoint `i`:

- If there is a next wp: `max(next.t_sec, arrive + wait)`.  
- Else: `arrive + max(wait, 0.1)`.

#### `earliest_conflict(a, b, eps)`

Nested loop over waypoints; same cell + overlapping `[arrive, depart]`; keep hit with earliest `overlap_start`.

#### `solve(candidate, cache)` — main loop

```
working = candidate
others = cache.all_except(candidate.amr_id)

loop up to max_iters:
  find earliest conflict vs any other
  if none:
    upsert working (+ modified others)
    return Ok or UpdatedOther

  wait_self  = other_clear + margin - my_arrive
  wait_other = self_clear  + margin - other_arrive

  if neither feasible → FailReplan

  if self_can:          # FIFO preference
    delay candidate
  else:
    delay other; mark updated_others

if loop ends → FailReplan (max iters)
```

Wait formula:

```
wait = other_clear_time + safety_margin_sec − my_arrival_time
```

---

## 8. Optional standalone node

**Not used** when `EcbsGate` is enabled in the fleet adapter.

### `ecbs_node.hpp`

`EcbsNode : rclcpp::Node` with:

- Subs: path candidates (`PathRequest`), itinerary set  
- Pubs: `ToZoneManager`, `EcbsReplan`  
- Own `ScheduleCache` + `EcbsSolver`  
- Params: margins / iters / epsilon (defaults match core: margin **2.0**)

### `ecbs_node.cpp`

Converts `PathRequest` / itinerary into `TimedPath`, runs `solve`, publishes lean msgs (same idea as the gate, but over topics).

### `main.cpp`

```cpp
rclcpp::init → make EcbsNode → spin → shutdown
```

### `launch/ecbs_planner.launch.xml`

Args for topics + solver params; starts `ecbs_planner_node`.  
Legacy note in launch still mentions `/ecbs_path_candidates` from GoToPlace — production gate does **not** use that hop.

---

## 9. Unit tests — `test/test_ecbs_solver.cpp` (solver); other files cover conflict store, proposal engine, group scheduler, arbitration, config defaults

| Test | Asserts |
|------|---------|
| `ConflictFreePassesThrough` | No waits when paths do not clash |
| `LaterAmrWaitsWhenThatFixesConflict` | Candidate waits; others unchanged |
| `FifoPrefersCandidateWaitEvenIfOtherWaitIsSmaller` | (legacy name) see `test_ecbs_solver.cpp`; solver does not use FIFO |
| `EarlierAmrWaitsWhenOnlyThatFixesConflict` | Feasibility fallback → `UpdatedOther` |
| `LaterAmrRecordsWaitingFor` | `waiting_for` + clear points filled |
| `FailWhenNeitherCanWait` | `FailReplan` when `max_wait` too small |

Helpers: `wp(t,x,y,index)`, `make_path(id, pts)`.

---

## 10. How the fleet adapter uses this package

```
GoToPlace::_execute_plan(plan)
  → EcbsGate::process(...)
       → TimedPath from Plan
       → EcbsSolver::solve(candidate, ScheduleCache)   // this package
       → publish /to_zone_manager (lean msgs from this package)
  → ExecutePlan::make → … → /robot_path_requests
```

Link: `rmf_fleet_adapter` → `ecbs_planner::ecbs_planner_core` + generated msg headers.

---

## 11. Build / test this package alone

```bash
source /opt/ros/humble/setup.bash
cd <workspace root>
colcon build --packages-select ecbs_planner
source install/setup.bash
colcon test --packages-select ecbs_planner
```

---

## 12. Topics (when used)

| Topic | Type | Who publishes (production) |
|-------|------|----------------------------|
| `/to_zone_manager` | `ToZoneManager` | `EcbsGate` (fleet adapter), using these msgs |
| `/ecbs_replan_candidate` | `EcbsReplan` | `evaluate_path_server` (ReplanCoordinator + GlobalResolver) |
| `/ecbs_path_approval` | `std_msgs/String` (amr id) | `evaluate_path_server` — seeded path released once no REPLAN is pending |
| `/ecbs_path_candidates` | `PathRequest` | Optional standalone node only |

---

## 12a. Conflict detection + replanning in `evaluate_path_server`

Both server modes run the same replan check on the RMF handover
(`/from_zone_manager`), before the path is approved:

| Step | Legacy (`global_resolver_enabled: false`) | Resolver (`global_resolver_enabled: true`) |
|------|-------------------------------------------|--------------------------------------------|
| Conflict decisions | `evaluate_path` on `ScheduleCache` + `ConflictStore` validation of ZM register | `GlobalResolver::register_handover` (sticky, full-route) |
| Seed-time conflict solve | `register_from_zone_manager` (holds in `ScheduleCache`) | same, on a `ScheduleCache` mirror (`resolver_replan_enabled`) |
| Wait vs detour | `ReplanCoordinator::update` (seed, every evaluate, `replan_period_sec` timer) | `ReplanCoordinator::update` (seed + timer) on the mirror |
| Approval | `/ecbs_path_approval` after `approval_settle_sec` unless REPLAN | same; without replan check → approved at once |
| `/to_zone_manager` | `ScheduleCache` paths | resolver decided holds only (the mirror never publishes) |

In resolver mode the mirror only decides REPLAN; waits / releases stay with
the GlobalResolver. A REPLAN closes `blocked_waypoints` for the candidate, the
fleet adapter plans around them and sends a new handover, which the resolver
re-decides. Set `resolver_replan_enabled: false` to publish only the
resolver's unresolvable-conflict report.

---

## 13. Out of scope inside this package

- Publishing `/robot_path_requests`  
- VDA / execution  
- `base_horizon`  
- Edge conflicts / full CBS tree search (vertex wait-only only)

---

## Strict base booking (hard rule)

A base booked by a moving AMR — from `base_tail_m` behind its pose to
its base end — belongs to it alone. Any other AMR whose path touches that
stretch at even one waypoint gets `WAIT` (`safety_distance_m` before it, no
register needed), **whatever the timing**. Unchanged: first reserver keeps the
cells (FIFO), followers trail at `safety_distance_m`, standing/waiting AMRs
block only their body with the same `safety_distance_m` spacing, and all other
ECBS logic. Rear cells free as the owner moves on (`update_pose` trims the tail).
See `config/ecbs_params.yaml` / `SolverConfig` for current metre values.

## Junction base clip

A T / + junction **deep inside** the requested base (≥ `safety_distance_m`
before its end) gets the **full base** — no wait point before it. Only a
junction within `safety_distance_m` of the **base end** (just before or just
after it) cuts the base to `safety_distance_m` before the centre (WAIT labeled
`T` / `+`), so a robot waiting at the base end never sits in the junction.
That stop is released once the robot is within `junction_release_m` (8 m) of
it with no real conflict, and never returns once the base is given past it. A
real conflict wait that lands within `safety_distance_m` of a centre is still
moved back to `safety_distance_m` before it. The stop XY depends only on the
path and the junction, not on the robot pose, so it does not move or blink.
