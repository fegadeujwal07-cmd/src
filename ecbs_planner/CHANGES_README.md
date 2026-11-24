# ECBS Planner — short pointer

| Doc | Content |
|-----|---------|
| **[README.md](README.md)** | Full detail of **all `ecbs_planner` code** (msgs, cache, solver, node, tests, CMake) |
| **[../../CHANGES_DETAIL_README.md](../../CHANGES_DETAIL_README.md)** | Fleet-adapter edits (`Node`, `GoToPlace`, `EcbsGate`) line-by-line + system flow |

### Quick facts
- Library: `ecbs_planner_core` (schedule cache + wait-only solver)
- Msgs: lean `/to_zone_manager` + `/ecbs_replan_candidate`
- Policy: FIFO when both can wait; 2 s clearance default
- Production: used **in-process** by `EcbsGate` — standalone node optional
