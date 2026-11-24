# ECBS → TM: eligible passage proposal (NOT robot permission).

uint64 proposal_id
uint64 decision_id
string amr_id
string fleet_name
# Canonical route/task identity (planner / schedule seed). Commit this exact value.
string task_id
# Robot execution order / PathRequest.task_id (e.g. "1"); not used for route match.
string execution_order_id
int32 phase_index
uint32 path_revision
string route_hash

string action_id
uint64 group_id
uint64 group_structure_version
uint64 route_geometry_version
uint64 authority_version
uint64 occupancy_clearance_version
uint64 policy_priority_version

# Proposed movement range on the seeded route.
RouteOccurrence from_occurrence
RouteOccurrence to_occurrence
float64 proposed_horizon_m

string[] required_conflict_ids
string[] exit_conditions
WaitDependency[] waiting_constraints

string decision_reason
# WAIT_CLEARANCE | WAIT_SEARCH | WAIT_HARDHOLD when status=WAIT
string wait_reason_code
uint64 priority_version
uint64 route_version
uint64 occupancy_version
uint64 commitment_version

# FREE_PROPOSAL | WAIT | STALE_ROUTE | RECOVERY_REQUIRED | AUTHORITY_CONFLICT
string status
