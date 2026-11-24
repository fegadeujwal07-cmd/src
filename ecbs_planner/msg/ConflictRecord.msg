# Persistent overlap bookkeeping identity (not a predefined zone).

string conflict_id
string[] participant_amrs
string fleet_name

# Per-participant route identity (parallel arrays with participant_amrs).
string[] task_ids
int32[] phase_indices
uint32[] path_revisions

# Overlap geometry on each participant route (parallel with participant_amrs).
RouteOccurrence[] entry_occurrence
RouteOccurrence[] exit_occurrence
RouteOccurrence[] wait_occurrence

# Per-participant easiness (parallel with participant_amrs).
# blocks_other[i]: participant_amrs[i] remaining path blocks the peer.
# blocked_by_other[i]: peer remaining path blocks participant_amrs[i].
bool[] blocks_other
bool[] blocked_by_other

string topology
string wait_reason

# Directed dependency: waiter waits for blocker on this overlap.
bool mandatory
string waiter_amr
string blocker_amr
string clearance_condition

uint64 source_route_version
uint64 source_occupancy_version
uint64 source_priority_version
uint64 source_commitment_version
uint64 record_version

bool tentative
bool physically_cleared
