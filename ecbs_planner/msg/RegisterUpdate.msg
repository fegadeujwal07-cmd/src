# One AMR wait decision from Zone Manager (batch register).
# Only WAITING AMRs are listed. GO AMRs are implied (blockers in waiting_for)
# and are not required in this list — ECBS clears stale waits on them.
string amr_id
string task_id
int32 phase_index
int32 path_revision
# Who this AMR waits for + where/how long. Must be non-empty for a waiter.
WaitDependency[] waiting_for
