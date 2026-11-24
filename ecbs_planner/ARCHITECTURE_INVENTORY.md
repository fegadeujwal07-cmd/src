# ECBS–TM Interface Inventory (frozen 2026-11-16)

Verified against headers, msgs, srv, callers — not README claims.

## Data flow

```
GoToPlace Plan
  → EcbsGate PUB /from_zone_manager (FromZoneManager) → evaluate_path_server
  → full_control PUB /traffic_manager/path_intent (PathIntent) → traffic_manager
       → CLIENT /evaluate_path_service (EvaluatePath)
       → PUB /traffic_manager/path_release (PathRelease) → full_control
            → PUB /robot_path_requests → execution_full_control
                 → PUB /BaseRequests → traffic_manager
                 → PUB /Monitored_task_details (priority) → TM / zone_monitor
```

Planner fans out routes in parallel (seed + intent). TM must not become the sole route distributor.

## Ownership today → target

| Concern | Today | Target |
|---------|-------|--------|
| Proposed routes | Planner | Planner (unchanged) |
| Conflict discovery | ECBS horizon evaluate | ECBS full-route persistent |
| Arbitration / GO set | **TM** `arbitrate_conflict` | **ECBS only** |
| Committed grants | TM PathGate | TM (validate ECBS proposal) |
| Priority metadata | execution → `/Monitored_task_details` | ECBS subscribes directly |
| PathRequest publish | full_control after PathRelease | unchanged |

## Key symbols

- ECBS: `evaluate_path`, `register_from_zone_manager`, `ScheduleCache`, `path_easiness_flags`
- TM: `PathGate`, `arbitrate_conflict`, `ConflictSessionStore`, `apply_conflict_arbitration`
- Priority: `execution::MonitoredTaskManager` publishes at standby/underway (before motion)

## Defects addressed by migration

1. TM independently picks winners
2. Horizon-scoped evaluate drops far conflicts
3. No persistent ConflictRecord / occurrence addressing
4. Evaluate re-ships full waypoints
5. Priority consumed by TM, not ECBS
6. No proposal/commit/AUTHORITY_CONFLICT protocol
7. TM launch was commented out in `execution_full_control.launch.py`
