# TM → ECBS: authoritative grant lifecycle event.

builtin_interfaces/Time stamp
uint64 grant_id
uint64 proposal_id
uint64 decision_id

string amr_id
string fleet_name
string task_id
int32 phase_index
uint32 path_revision

# COMMITTED | REJECTED | STALE | SUPERSEDED | COMPLETED
string status
string reason

RouteOccurrence from_occurrence
RouteOccurrence to_occurrence
int32 release_through_index

uint64 route_version
uint64 occupancy_version
uint64 commitment_version
