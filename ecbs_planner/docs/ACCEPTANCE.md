# Live acceptance tests still required

Unit tests (`colcon test --packages-select ecbs_planner`) cover the logic. These
four checks need the running system (ECBS server + traffic manager + fleet
adapter / simulator) and are **not** automated yet.

1. **Two-robot happy path with real task ids.**
   Two robots, non-conflicting or compatible routes.
   `BaseExtensionQuery` → `FREE_PROPOSAL` → TM `CommitProposal` → `ACCEPTED` →
   `PathRelease`. The task id carried through must be the planner/dispatch UUID,
   different from the execution order id (`"1"`). Check `MovementProposal.task_id`,
   the commit request and the grant in `proposal_state_` all use the UUID.

2. **Racing incompatible commits: exactly one wins.**
   Two robots with an exclusive crossing both get a `FREE_PROPOSAL` for the
   passage and TM commits both at the same time. Exactly one `ACCEPTED`; the
   other must be `REJECTED_STALE` (conflict record / authority changed) or
   `REJECTED_MISMATCH`, never two grants over the same overlap. Repeat with 10–30
   robots: unrelated robots' path changes or clearances must not reject a commit
   (F4 scoped staleness).

3. **Idle robot on an exit: the winner holds before entering.**
   Park an idle robot on the exit of an overlap. The robot that would pass must
   get a hold before the entry (`exit_occupied`, `WAIT` with `waiting_for`), never
   a grant into the overlap. A peer whose route is compatible with the held robot
   must still be released; one whose route is not gets `WAIT_HARDHOLD` (F6).

4. **Replay determinism under the node-expansion budget.**
   Record a session (paths, priorities, clearances, commits), replay it twice with
   `scheduler_max_nodes` small enough to force `WAIT_SEARCH` resumption. Both runs
   must produce the same proposals, wait reasons and grant order. Wall-clock
   budget stays off (`use_wall_clock=false`).

Also verify live (new in F3/F7): conflict ids and group ids stay unchanged while
a robot drives through an overlap (`ConflictRecord.conflict_id` /
`MovementProposal.group_id`), and a `register` with a wait inside the overlap, a
non-partner blocker, or a T/+ centre returns `success=false` with no cache change.
