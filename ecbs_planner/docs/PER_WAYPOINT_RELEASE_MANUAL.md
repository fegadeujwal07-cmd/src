# (B) Per-waypoint lane release — manual test plan

`EcbsReplan.msg` already carries parallel `blocked_waypoints` / `blocked_until_sec`.
`full_control` still does `open_at = max(blocked_until_sec)` and reopens **all**
closed lanes on one timer.

## Checklist
1. Dispatch a head-on pair so `/ecbs_replan_candidate` publishes REPLAN with
   staggered `blocked_until_sec` values (time-filtered corridor).
2. Confirm `full_control` closes every lane touching those waypoints once.
3. Confirm lanes all reopen together at `max(until)`, not per index.
4. When field needs staggered open: change the EcbsReplan handler to track
   per-waypoint (or per-lane) expiry instead of a single `ecbs_open_at`.
