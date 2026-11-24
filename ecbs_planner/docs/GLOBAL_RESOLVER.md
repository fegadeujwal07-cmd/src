# GlobalResolver — ECBS as the single traffic authority

Code: `include/ecbs_planner/global_resolver.hpp`, `src/global_resolver.cpp`,
wired in `src/evaluate_path_server_main.cpp`. Tests: `test/test_global_resolver.cpp`.

## Responsibilities

| Component | Does | Does not |
|---|---|---|
| RMF core | plans the path; `GoToPlace::_execute_plan` → `EcbsGate` forwards the waypoints on `/from_zone_manager` **before** `ExecutePlan::make` | resolve traffic (gate's local solve is skipped with the external server) |
| ECBS (`evaluate_path_server`) | on that **RMF handover**: stores the route, finds every overlap on the **full remaining routes**, decides order + safe wait points immediately; releases waiters from `/fleet_states` progress | — |
| traffic_manager | registers its PathIntent (binds to the handed-over route: same `fleet\|task\|phase\|rev` → decisions kept, waypoint numbering remapped), asks for the next release, commits it, publishes PathRelease | decide who goes first (`ecbs_authoritative: true` disables its peer-base veto) |
| zone_manager | unchanged runtime safety fallback | — |

## Flow

1. RMF hands the plan over → `EcbsGate` → `/from_zone_manager` →
   `GlobalResolver::register_handover` (`[ECBS-HANDOVER]`).
2. Every overlap with every other robot is found and decided immediately
   (`[ECBS-DECIDE]` log line explains first robot, waiter, wait point, release point, rule).
   A stale copy of an older route of that robot is ignored.
2b. full_control → PathIntent → TM → `EvaluatePath` with waypoints → `register_intent`:
   same identity + geometry → **bound** (no re-decision); geometry differs → re-decided on
   the executed PathIntent (`[ECBS-WARN]`). Strict mode (`decide_only_on_rmf_handover`):
   a PathIntent arriving before its handover is **parked** (robot WAITs) and binds when the
   handover is evaluated; only after `handover_wait_timeout_sec` without a handover is it
   decided on the PathIntent (`[ECBS-WARN]`). TM queries never trigger resolution.
   A TM query from an older session after RMF handed over a new revision → WAIT at its tip.
3. TM `BaseExtensionQuery` → release up to `query_horizon_m`, capped at decided wait
   points and never within `clearance_m` of another robot's occupied base (no foreign base).
   `FREE_PROPOSAL` → TM commits (`CommitProposal`, re-validated) → PathRelease.
   `WAIT` → robot holds at its tip (= wait point). `ROUTE_COMPLETE` → nothing to release.
4. `/fleet_states` progress: when the first robot is `safety_distance_m` past the
   overlap exit (same direction: past entry + safety + clearance), `[ECBS-CLEAR]` is logged and
   `/ecbs/release_event` wakes the waiter in TM, which re-queries at once.

## Rulebook (per overlap, strongest first)

| Rule | Order |
|---|---|
| R1 feasibility | robot that cannot hold safely before the overlap (starts inside / base already in the approach) goes first. If neither can: the one **standing on the other's path** moves first (`unblock`, e.g. robots parked nose-to-tail — the rear one holds in place); neither on the other's path → further-in first (`committed`); only face-to-face (both on each other's path) → unresolvable → `/ecbs_replan_candidate`, re-checked every 2 s (`[ECBS-RECOVER]`) |
| R2 blocking | robot whose route ends inside the overlap (would park on the other's path) goes second |
| health | healthy robot first |
| R3 same direction | robot ahead leads; follower trails `safety_distance_m` behind its base |
| dependency | robot already held for another robot before it reaches this overlap does not go first (prevents chain waits like amr5→amr2→amr1→amr5) |
| near (crossing) | robot already at the crossing approach first |
| free pass | robot that clears before the other reaches its wait point goes first (`resolver_free_pass_ratio`) |
| priority → load → eta → fifo → id | zone_manager ranking |
| group | passage dependency graph: a wait blocks the waiter's first passage after its wait point; a same-direction leader blocks its follower until it leaves the section; plus route order. A cycle = deadlock → weakest non-forced decision flipped, else unresolvable |

Safe wait point: the **nearest** route waypoint ≥ `safety_distance_m` before the overlap entry,
never behind the robot's committed position, off the blocker's path and blocking no other robot.
A spot on another robot R's route is accepted when the decided order already keeps them apart:
R passes first and the waiter is held before that spot until R is released past it; or R is held at
its own wait point behind the spot until the waiter has passed; or (same direction, inside the shared
section) R follows behind the waiter / leads in front of it. A junction-free spot is preferred only
within `junction_clear_m + safety_distance_m` further back. Fallbacks: a spot clear only of the
blocker (relaxed), then **hold in place** at the committed waypoint when it is still outside the
overlap and off the blocker's path. Once all decisions of a handover exist, and every ~2 s with live
progress, pending waits are moved up to the nearest such waypoint (`[ECBS-WAIT] … now holds at wpN
(was wpM)`); the order is never changed and a wait never moves behind the granted tip.

Decisions are **sticky**: keyed by route geometry, never re-decided because a robot moved;
re-registering an identical route is a no-op.

## Parameters

`config/ecbs_params.yaml`: `global_resolver_enabled`, `register_on_rmf_handover`,
`decide_only_on_rmf_handover`, `handover_wait_timeout_sec`,
`resolver_free_pass_ratio`, `resolver_crossing_max_len_m`, `resolver_body_back_m`,
`release_event_topic`, `replan_candidate_topic` (+ `clearance_m`, `safety_distance_m`).

`traffic_manager.yaml`: `ecbs_authoritative`, `ecbs_release_event_topic`,
`recovery_min_interval_sec`, `held_reevaluate_sec` (safety-net poll).

## Rollback

`global_resolver_enabled: false` (ECBS) and `ecbs_authoritative: false` (TM) restore the
previous proposal-engine path unchanged.
