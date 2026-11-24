# traffic_manager → full_control: which prefix of path may be published as PathRequest.

builtin_interfaces/Time stamp

string fleet_name
string robot_name
uint64 request_id

string task_id
string dispatch_task_id
uint32 phase_index
uint32 path_revision

# false → do not publish PathRequest for this request_id.
bool approved

# Contiguous released indices [0 .. release_through_index]. -1 if none.
int32 release_through_index

# Monotonic per PathIntent session (0 = first release). full_control maps to PathRequest.path_revision offset.
uint32 release_update_id

# ECBS proposal/hold snapshot for this release.
string ecbs_status
int32 hold_leg_index
float64 hold_x
float64 hold_y
float64 hold_wait_sec
string[] blocking_amr_ids
