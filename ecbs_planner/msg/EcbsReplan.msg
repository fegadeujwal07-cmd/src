# /ecbs_replan_candidate: evaluate_path_server's decision for a head-on pair
# that waiting cannot resolve (each AMR holds for the other on opposite paths).
builtin_interfaces/Time stamp  # When ECBS decided
string amr_id                  # Candidate AMR: replans around keep_amr's corridor
string task_id                 # Candidate's task id (may be empty)
string reason                  # Human-readable cause / selection reason
string[] conflicting_amrs      # AMRs involved in the unresolved conflict

string DECISION_REPLAN = "REPLAN"
string DECISION_NO_ALTERNATIVE = "NO_ALTERNATIVE"
string DECISION_CLEAR = "CLEAR"
# REPLAN: close blocked_waypoints for amr_id only (replaces its earlier set).
# NO_ALTERNATIVE: amr_id found no route around keep_amr (informational).
# CLEAR: reopen everything closed for amr_id by ECBS.
string decision
string keep_amr                # AMR that keeps its path
int64[] blocked_waypoints      # Nav-graph indices of keep_amr's remaining shared corridor
float64[] blocked_until_sec    # Planned time keep_amr leaves each blocked waypoint
uint32 revision                # Bumps whenever the decision or blocked set changes
